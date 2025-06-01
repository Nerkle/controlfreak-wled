#include "SerialControlUsermod.h"

extern WS2812FX strip;

#define SERIAL_BAUD       115200
#define CMD_BUFFER_SIZE   (5 * 1024)   // accept up to 5 KB of incoming JSON
#define SEG_DOC_CAP       256          // ~256 bytes to parse a single segment

// Two distinct response‐buffer sizes:
static const size_t RESP_SMALL_CAP = 256;   // for setState “ok” / error messages
static const size_t RESP_LARGE_CAP = (5 * 1024);  // for getState full segment dump

void SerialControlUsermod::setup() {
    // Begin USB‐Serial for debug output
    Serial.begin(115200);
    Serial.println("SerialControlUsermod: starting up");

    DEBUG_PRINTLN("▶ SerialControlUsermod::setup() called");

    // Initialize the “secondary” UART (Serial1) on GPIO16=RX1, GPIO17=TX1:
    Serial1.begin(SERIAL_BAUD, SERIAL_8N1, /*RX=*/16, /*TX=*/17);
    Serial1.setTimeout(10);

    Serial.println(F("SerialControlUsermod: listening on Serial1 for JSON commands…"));
}

void SerialControlUsermod::loop() {
    // Read incoming bytes from Serial1 until newline
    while (Serial1.available()) {
        char c = (char)Serial1.read();
        if (c == '\n') {
            // We have a complete JSON command in inputLine
            handleCommand(inputLine);
            inputLine.clear();
        } else if (c != '\r') {
            inputLine += c;
            // Safety: if buffer grows too large, reset it
            if (inputLine.length() > CMD_BUFFER_SIZE - 1) {
                inputLine.clear();
                Serial.println(F("SerialControlUsermod: buffer overflow, dropping line"));
            }
        }
    }
}

void SerialControlUsermod::handleCommand(const String &cmdLine) {
    // 1) Parse only the top‐level "cmd" field into a small StaticJsonDocument
    StaticJsonDocument<128> headerDoc;
    DeserializationError hdrErr = deserializeJson(headerDoc, cmdLine);
    if (hdrErr) {
        StaticJsonDocument<RESP_SMALL_CAP> respErr;
        respErr["status"]  = "error";
        respErr["message"] = "invalid JSON";
        String out;
        serializeJson(respErr, out);
        Serial1.println(out);
        return;
    }

    const char *cmd = headerDoc["cmd"];
    if (!cmd) {
        StaticJsonDocument<RESP_SMALL_CAP> respNoCmd;
        respNoCmd["status"]  = "error";
        respNoCmd["message"] = "missing cmd";
        String out;
        serializeJson(respNoCmd, out);
        Serial1.println(out);
        return;
    }

    // 2) Handle "setState" → small response buffer
    if (strcmp(cmd, "setState") == 0) {
        bool anyChange = false;

        // 2.a) Find the substring "\"seg\":[" in cmdLine
        int idxSegKey = cmdLine.indexOf("\"seg\":");
        if (idxSegKey < 0) {
            // No "seg" → nothing to change
            StaticJsonDocument<RESP_SMALL_CAP> respOK;
            respOK["status"]   = "ok";
            respOK["segCount"] = strip.getSegmentsNum();
            String out;
            serializeJson(respOK, out);
            Serial1.println(out);
            return;
        }

        // 2.b) Find '[' after "\"seg\":"
        int idxOpenBracket = cmdLine.indexOf('[', idxSegKey);
        if (idxOpenBracket < 0) {
            StaticJsonDocument<RESP_SMALL_CAP> respErr;
            respErr["status"]  = "error";
            respErr["message"] = "malformed seg array";
            String out;
            serializeJson(respErr, out);
            Serial1.println(out);
            return;
        }

        // 2.c) Find matching ']' by tracking nesting
        int depth = 1;
        int idxCloseBracket = -1;
        for (int i = idxOpenBracket + 1; i < cmdLine.length(); i++) {
            char ch = cmdLine.charAt(i);
            if (ch == '[')  depth++;
            else if (ch == ']') {
                depth--;
                if (depth == 0) {
                    idxCloseBracket = i;
                    break;
                }
            }
        }
        if (idxCloseBracket < 0) {
            StaticJsonDocument<RESP_SMALL_CAP> respErr;
            respErr["status"]  = "error";
            respErr["message"] = "unterminated seg array";
            String out;
            serializeJson(respErr, out);
            Serial1.println(out);
            return;
        }

        // 2.d) Extract exactly the substring "[ {…}, {…}, … ]"
        String segArrayJson = cmdLine.substring(idxOpenBracket, idxCloseBracket + 1);

        // 2.e) Clear all existing segments
        strip.resetSegments();

        // 2.f) Walk through segArrayJson looking for each "{ … }"
        int len = segArrayJson.length();
        int i = 0;
        uint8_t newSegCount = 0;
        while (i < len) {
            // 2.f.i) Skip until next '{'
            if (segArrayJson.charAt(i) != '{') {
                i++;
                continue;
            }
            // 2.f.ii) Found '{' → find matching '}'
            int braceDepth = 1;
            int j = i + 1;
            for (; j < len; j++) {
                char c2 = segArrayJson.charAt(j);
                if (c2 == '{')      braceDepth++;
                else if (c2 == '}') braceDepth--;
                if (braceDepth == 0) break;
            }
            if (j >= len) {
                // Unterminated '{'
                break;
            }

            // 2.f.iii) Extract oneSegJson = substring(i..j)
            String oneSegJson = segArrayJson.substring(i, j + 1);
            i = j + 1; // advance for next iteration

            // 2.f.iv) Deserialize just this one segment object
            StaticJsonDocument<SEG_DOC_CAP> segDoc;
            auto segErr = deserializeJson(segDoc, oneSegJson);
            if (segErr) {
                Serial.printf("⚠️ SerialControlUsermod: failed to parse single seg: %s\n",
                              segErr.c_str());
                continue;
            }
            JsonObject sObj = segDoc.as<JsonObject>();

            // 2.f.v) Pull fields (with defaults)
            bool segOn      = sObj["on"]   | true;   // default true
            uint16_t start  = sObj["start"] | 0;
            uint16_t stop   = sObj["stop"]  | 0;
            uint8_t fx      = sObj["fx"]    | 0;
            uint8_t sx      = sObj["sx"]    | 150;   // default speed
            uint8_t ix      = sObj["ix"]    | 150;   // default intensity

            if (stop < start) {
                // invalid range → skip
                continue;
            }

            // 2.f.vi) Create the segment
            strip.setSegment(newSegCount, start, stop,
                             1, 0, UINT16_MAX, 0, 1);

            // 2.f.vii) Assign effect (mode)
            strip.setMode(newSegCount, fx);

            // 2.f.viii) Assign speed & intensity
            strip.getSegment(newSegCount).speed     = sx;
            strip.getSegment(newSegCount).intensity = ix;

            // 2.f.ix) If “col” array exists, load up to 3 color slots
            if (sObj.containsKey("col") && sObj["col"].is<JsonArray>()) {
                JsonArray colIn = sObj["col"].as<JsonArray>();
                for (uint8_t cIndex = 0; cIndex < 3 && cIndex < colIn.size(); cIndex++) {
                    JsonArray trip = colIn[cIndex].as<JsonArray>();
                    if (trip.size() == 3) {
                        uint8_t r = trip[0].as<uint8_t>();
                        uint8_t g = trip[1].as<uint8_t>();
                        uint8_t b = trip[2].as<uint8_t>();
                        strip.getSegment(newSegCount).setColor(
                          cIndex, RGBW32(r, g, b, 0));
                    }
                }
            }

            newSegCount++;
            anyChange = true;
        }

        // 2.g) Finish segment setup
        strip.setMainSegmentId(0);
        strip.setTargetFps(WLED_FPS);
        strip.setShowCallback(nullptr);
        strip.setTransitionMode(false);
        strip.restartRuntime();

        // 2.h) If any segment changed, force immediate update
        if (anyChange) {
            strip.service();
        }

        // 2.i) Send back a small “ok” JSON
        StaticJsonDocument<RESP_SMALL_CAP> respOK;
        respOK["status"]   = "ok";
        respOK["segCount"] = strip.getSegmentsNum();
        String outOK;
        serializeJson(respOK, outOK);
        Serial1.println(outOK);
        return;
    }

    // 3) Handle “getState” → larger response buffer
    if (strcmp(cmd, "getState") == 0) {
        StaticJsonDocument<RESP_LARGE_CAP> resp;
        JsonArray segArr = resp.createNestedArray("seg");

        for (uint8_t i = 0; i < strip.getSegmentsNum(); i++) {
            auto &S = strip.getSegment(i);
            JsonObject so = segArr.createNestedObject();
            so["id"]    = i;
            so["on"]    = S.on;
            so["start"] = S.start;
            so["stop"]  = S.stop;
            so["fx"]    = S.mode;          // effect index
            so["sx"]    = S.speed;         // speed
            so["ix"]    = S.intensity;     // intensity

            JsonArray colOut = so.createNestedArray("col");
            for (uint8_t cidx = 0; cidx < 3; cidx++) {
                CRGB c = S.colors[cidx];
                JsonArray rgb = colOut.createNestedArray();
                rgb.add(c.red);
                rgb.add(c.green);
                rgb.add(c.blue);
            }
        }

        String out;
        serializeJson(resp, out);
        Serial1.println(out);
        return;
    }

    // 4) Unknown “cmd”
    StaticJsonDocument<RESP_SMALL_CAP> respUnknown;
    respUnknown["status"]  = "error";
    respUnknown["message"] = "unknown cmd";
    String outUnknown;
    serializeJson(respUnknown, outUnknown);
    Serial1.println(outUnknown);
}