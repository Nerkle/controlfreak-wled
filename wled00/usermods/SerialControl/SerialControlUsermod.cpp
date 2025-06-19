#include "SerialControlUsermod.h"
#include "wled.h"

extern WS2812FX strip;
// extern BusConfig* busConfigs[WLED_MAX_BUSSES + WLED_MIN_VIRTUAL_BUSSES];

#define SERIAL_BAUD       115200
#define CMD_BUFFER_SIZE   (5 * 1024)   // accept up to 5 KB of incoming JSON
#define SEG_DOC_CAP       512          // ~256 bytes to parse a single segment

// Two distinct response‐buffer sizes:
static const size_t RESP_SMALL_CAP = 256;   // for setState “ok” / error messages
static const size_t RESP_LARGE_CAP = (5 * 1024);  // for getState full segment dump

void SerialControlUsermod::setup() {
    Serial.begin(115200);
    Serial.println("blah");
    Serial.println("SerialControlUsermod: starting up");
    DEBUG_PRINTLN("▶ SerialControlUsermod::setup() called");

    // // // GPIO pins for each strip
    // uint8_t pins1[] = { 4 };   // first output GPIO
    // uint8_t pins2[] = { 2 };   // second output GPIO
    // uint8_t pins3[] = { 23 };   // third output GPIO

    // BusManager::setMilliampsMax(0);
    //
    // busConfigs[0] = new BusConfig(
    //   TYPE_WS2812_RGB,
    //   pins1,
    //   0,
    //   800,
    //   COL_ORDER_BRG
    // );
    // busConfigs[0]->milliAmpsPerLed = 55;
    // busConfigs[0]->milliAmpsMax    = 10000;
    //
    // busConfigs[1] = new BusConfig(
    //     TYPE_WS2812_RGB,
    //     pins2,
    //     800,   // next pixel offset
    //     800,
    //     COL_ORDER_BRG
    // );
    // busConfigs[1]->milliAmpsPerLed = 55;
    // busConfigs[1]->milliAmpsMax    = 10000;
    //
    // busConfigs[2] = new BusConfig(
    //     TYPE_WS2812_RGB,
    //     pins3,
    //     1600,  // next pixel offset
    //     800,
    //     COL_ORDER_BRG
    // );
    // busConfigs[2]->milliAmpsPerLed = 55;
    // busConfigs[2]->milliAmpsMax    = 10000;
    //
    // // mark that WLED should re‐initialize all buses
    // doInitBusses = true;

    Serial1.begin(SERIAL_BAUD, SERIAL_8N1, /*RX=*/16, /*TX=*/17);
    Serial1.setTimeout(500);
    Serial1.setRxBufferSize(2048);
    Serial.println(F("SerialControlUsermod: listening on Serial1 for JSON commands…"));
}

void SerialControlUsermod::loop() {
    static bool awaitingCmd = true;        // Searching for prefix?
    static size_t matchIndex = 0;          // Prefix match progress
    static const char prefix[] = "{\"cmd\":";
    static const size_t prefixLen = sizeof(prefix) - 1;
    static int braceDepth = 0;
    static String inputLine;
    static bool inJson = false;
    static size_t chunkCount = 0;

    int avail0 = Serial1.available();
    if (!avail0) return;
    // Serial.printf("[UC] %d bytes available at loop start\n", avail0);

    while (Serial1.available()) {
        int b = Serial1.read();
        if (b < 0) return;
        char c = (char)b;
        // Serial.printf("[UC] got: 0x%02X '%c'\n", b, (c >= 32 && c < 127) ? c : '.');

        if (awaitingCmd) {
            // Debugging: show prefix matching progress
            // Serial.printf("[UC] matching prefix: index %u/%u, char '%c', expected '%c'\n",
            //               matchIndex, prefixLen, c, prefix[matchIndex]);
            if (c == prefix[matchIndex]) {
                matchIndex++;
                if (matchIndex == prefixLen) {
                    Serial.println("[UC] found prefix {\"cmd\":, starting JSON");
                    inputLine = String(prefix);
                    braceDepth = 1;   // The '{' from the prefix
                    inJson = true;
                    awaitingCmd = false;
                    chunkCount = 0;
                    matchIndex = 0;
                }
            } else {
                if (matchIndex && c == prefix[0]) {
                    Serial.println("[UC] partial prefix interrupted, found '{', restarting matchIndex at 1");
                    matchIndex = 1;
                } else {
                    if (matchIndex) Serial.println("[UC] prefix match lost, reset to 0");
                    matchIndex = 0;
                }
            }
        }
        else if (inJson) {
            inputLine += c;
            chunkCount++;
            // Serial.printf("[UC] added to inputLine, total len=%u\n", inputLine.length());

            if (c == '{')      { braceDepth++; /* Serial.printf("[UC] braceDepth++ = %d\n", braceDepth); */ }
            else if (c == '}') { braceDepth--; /* Serial.printf("[UC] braceDepth-- = %d\n", braceDepth); */ }

            // Only complete when braces closed AND newline seen
            if (braceDepth == 0 && c == '\n') {
                Serial.printf("[UC] complete JSON (%u bytes, %u read calls):\n%s\n",
                              inputLine.length(), chunkCount, inputLine.c_str());
                handleCommand(inputLine);
                inputLine = "";
                inJson = false;
                awaitingCmd = true;  // Start looking for next command
                chunkCount = 0;
                Serial.println("[UC] Command parsed and handler called, returning to prefix search");
                break;
            }
            if (inputLine.length() > CMD_BUFFER_SIZE - 1) {
                Serial.println("[UC] buffer overrun, clearing");
                inputLine = "";
                inJson = false;
                awaitingCmd = true;
                chunkCount = 0;
            }
        }
    }
}

void SerialControlUsermod::handleCommand(const String &cmdLine) {
    Serial.println("handling Command:");
    Serial.println(cmdLine);

    // 1) Parse only the top‐level "cmd" field into a small StaticJsonDocument
    DynamicJsonDocument doc(cmdLine.length() * 4);
    DeserializationError hdrErr = deserializeJson(doc, cmdLine);
    if (hdrErr) {
        Serial.print("hdrErr = ");
        Serial.println(hdrErr.c_str());

        StaticJsonDocument<RESP_SMALL_CAP> respErr;
        respErr["status"]  = "error";
        respErr["message"] = "invalid JSON";
        String out;
        serializeJson(respErr, out);
        Serial.println("response (1):");
        Serial.println(out);
        Serial1.println(out);
        return;
    }
    Serial.println("passed error...");

    const char *cmd = doc["cmd"];
    if (!cmd) {
        StaticJsonDocument<RESP_SMALL_CAP> respNoCmd;
        respNoCmd["status"]  = "error";
        respNoCmd["message"] = "missing cmd";
        String out;
        serializeJson(respNoCmd, out);
        Serial.println("response (2):");
        Serial.println(out);
        Serial1.println(out);
        return;
    }

    Serial.println("parsed command...");

    // 2) Handle "setState" → small response buffer
    if (strcmp(cmd, "setState") == 0) {
        Serial.println("Performing setState...");
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
            Serial.println("response (3):");
            Serial.println(out);
            // Serial1.println(out);
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
            Serial.println("response (4):");
            Serial.println(out);
            // Serial1.println(out);
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
            Serial.println("response (5):");
            Serial.println(out);
            // Serial1.println(out);
            return;
        }

        // 2.d) Extract exactly the substring "[ {…}, {…}, … ]"
        String segArrayJson = cmdLine.substring(idxOpenBracket, idxCloseBracket + 1);

        // 2.e) Clear all existing segments
        strip.resetSegments();

        // 2.f) Walk through segArrayJson looking for each "{ … }"
        int len = segArrayJson.length();
        Serial.printf("Processing segments: %s", segArrayJson.c_str());
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
            strip.getSegment(newSegCount).on        = segOn;
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

        Serial.println("Finishing processing segments");
        // 2.g) Finish segment setup
        strip.setMainSegmentId(0);
        strip.setTargetFps(WLED_FPS);
        strip.setShowCallback(nullptr);
        strip.setTransitionMode(false);
        strip.restartRuntime();

        // 2.h) If any segment changed, force immediate update
        if (anyChange) {
            Serial.println("Trigger strip change!");
            strip.service();
        }

        // 2.i) Send back a small “ok” JSON
        StaticJsonDocument<RESP_SMALL_CAP> respOK;
        respOK["status"]   = "ok";
        respOK["segCount"] = strip.getSegmentsNum();
        String outOK;
        serializeJson(respOK, outOK);
        Serial.println("response (6):");
        Serial.println(outOK);
        // Serial1.println(outOK);
        return;
    }

    // 3) Handle “getState” → larger response buffer
    if (strcmp(cmd, "getState") == 0) {
        Serial.println("Performing getState...");
        Serial.printf("Heap before JSON doc: %u\n", ESP.getFreeHeap());

        StaticJsonDocument<RESP_LARGE_CAP> resp;
        JsonArray segArr = resp.createNestedArray("seg");

        uint8_t numSegs = strip.getSegmentsNum();
        Serial.printf("getSegmentsNum() = %u\n", numSegs);

        for (uint8_t i = 0; i < numSegs; i++) {
            Serial.printf("About to fetch segment %u...\n", i);
            auto &S = strip.getSegment(i);

            Serial.printf("Segment %u: start=%u stop=%u on=%u mode=%u speed=%u intensity=%u colors@%p\n",
                          i, S.start, S.stop, S.on, S.mode, S.speed, S.intensity, (void*)S.colors);

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
                Serial.printf("Segment %u color[%u]: ", i, cidx);
                CRGB c = S.colors[cidx];
                Serial.printf("R=%u G=%u B=%u\n", c.red, c.green, c.blue);
                JsonArray rgb = colOut.createNestedArray();
                rgb.add(c.red);
                rgb.add(c.green);
                rgb.add(c.blue);
            }
        }

        Serial.printf("Heap after JSON doc: %u\n", ESP.getFreeHeap());

        String out;
        serializeJson(resp, out);
        Serial.println("response (7):");
        Serial.println(out);
        Serial1.println(out);
        return;
    }

    // 4) Unknown “cmd”
    StaticJsonDocument<RESP_SMALL_CAP> respUnknown;
    respUnknown["status"]  = "error";
    respUnknown["message"] = "unknown cmd";
    String outUnknown;
    serializeJson(respUnknown, outUnknown);
    Serial.println("response (8):");
    Serial.println(outUnknown);
    Serial1.println(outUnknown);
}
