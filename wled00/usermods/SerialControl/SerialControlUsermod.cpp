#include "SerialControlUsermod.h"
#include "wled.h"

extern WS2812FX strip;
extern BusConfig* busConfigs[WLED_MAX_BUSSES + WLED_MIN_VIRTUAL_BUSSES];
static std::vector<uint32_t> g_busBase;   // base pixel index for each bus
static bool g_topologyConfigured = false;

#define SERIAL_BAUD       115200
#define CMD_BUFFER_SIZE   (5 * 1024)   // accept up to 5 KB of incoming JSON
#define SEG_DOC_CAP       512          // ~256 bytes to parse a single segment

// Two distinct response‐buffer sizes:
static const size_t RESP_SMALL_CAP = 256;   // for setState “ok” / error messages
static const size_t RESP_LARGE_CAP = (5 * 1024);  // for getState full segment dump

void SerialControlUsermod::setup() {
    // Running this again here causes a crash
    // Serial.begin(115200);
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

static void respondOk(const char* key = nullptr, int value = -1) {
    StaticJsonDocument<256> doc;
    doc["status"] = "ok";
    if (key) doc[key] = value;
    String out; serializeJson(doc, out);
    Serial.println(out);
    Serial1.println(out);
}

static void respondError(const char* msg) {
    StaticJsonDocument<256> doc;
    doc["status"]  = "error";
    doc["message"] = msg ? msg : "error";
    String out; serializeJson(doc, out);
    Serial.println(out);
    Serial1.println(out);
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

    // 2) Handle "setState" → small response buffer
    if (strcmp(cmd, "setState") == 0) {
        if (!g_topologyConfigured) {
            // not fatal, but warn—your MCU should call setTopology at boot
            Serial.println(F("setState: topology not configured yet (continuing)"));
        }

        const char *mode = doc["mode"] | "replace"; // "replace" (default) or "patch"
        int32_t busOffset = doc["busOffset"] | 0; // add to each start/stop (relative addressing)

        JsonArray segArr = doc["seg"].as<JsonArray>();
        if (segArr.isNull()) {
            respondOk("segCount", strip.getSegmentsNum()); // nothing to do
            return;
        }

        bool anyChange = false;

        if (!strcmp(mode, "replace")) {
            strip.resetSegments();
            uint8_t newId = 0;

            for (JsonObject sObj: segArr) {
                bool on = sObj["on"] | true;
                int32_t st = sObj["start"] | 0;
                int32_t sp = sObj["stop"] | 0;
                int32_t fx = sObj["fx"] | 0;
                int32_t sx = sObj["sx"] | 150;
                int32_t ix = sObj["ix"] | 150;
                int32_t bri = sObj["bri"] | 255;

                st += busOffset;
                sp += busOffset;
                if (sp <= st) continue;

                strip.setSegment(newId, (uint16_t) st, (uint16_t) sp, 1, 0, UINT16_MAX, 0, 1);
                strip.setMode(newId, (uint8_t) fx);

                auto &S = strip.getSegment(newId);
                S.on = on;
                S.speed = (uint8_t) sx;
                S.intensity = (uint8_t) ix;
                S.setOpacity((uint8_t) bri);

                if (sObj.containsKey("col") && sObj["col"].is<JsonArray>()) {
                    JsonArray colIn = sObj["col"].as<JsonArray>();
                    for (uint8_t cIndex = 0; cIndex < 3 && cIndex < colIn.size(); cIndex++) {
                        JsonArray trip = colIn[cIndex].as<JsonArray>();
                        if (trip.size() == 3) {
                            uint8_t r = trip[0].as<uint8_t>();
                            uint8_t g = trip[1].as<uint8_t>();
                            uint8_t b = trip[2].as<uint8_t>();
                            S.setColor(cIndex, RGBW32(r, g, b, 0));
                        }
                    }
                }

                newId++;
                anyChange = true;
            }

            strip.setMainSegmentId(0);
            strip.setTargetFps(WLED_FPS);
            strip.setShowCallback(nullptr);
            strip.setTransitionMode(false);
            strip.restartRuntime();
        } else {
            // PATCH: update selected segments by id, or append
            for (JsonObject sObj: segArr) {
                int hasId = sObj.containsKey("id") ? sObj["id"].as<int>() : -1;

                bool on = sObj["on"] | true;
                int32_t st = sObj["start"] | -1;
                int32_t sp = sObj["stop"] | -1;
                int32_t fx = sObj["fx"] | -1;
                int32_t sx = sObj["sx"] | -1;
                int32_t ix = sObj["ix"] | -1;
                int32_t bri = sObj["bri"] | -1;

                if (st >= 0) st += busOffset;
                if (sp >= 0) sp += busOffset;
                if (st >= 0 && sp >= 0 && sp <= st) continue;

                if (hasId >= 0 && hasId < strip.getSegmentsNum()) {
                    auto &S = strip.getSegment((uint8_t) hasId);
                    if (st >= 0 && sp >= 0) strip.setSegment((uint8_t) hasId, (uint16_t) st, (uint16_t) sp, 1, 0,
                                                             UINT16_MAX, 0, 1);
                    if (fx >= 0) strip.setMode((uint8_t) hasId, (uint8_t) fx);
                    S.on = on;
                    if (sx >= 0) S.speed = (uint8_t) sx;
                    if (ix >= 0) S.intensity = (uint8_t) ix;
                    if (bri >= 0) S.setOpacity((uint8_t) bri);

                    if (sObj.containsKey("col") && sObj["col"].is<JsonArray>()) {
                        JsonArray colIn = sObj["col"].as<JsonArray>();
                        for (uint8_t cIndex = 0; cIndex < 3 && cIndex < colIn.size(); cIndex++) {
                            JsonArray trip = colIn[cIndex].as<JsonArray>();
                            if (trip.size() == 3) {
                                uint8_t r = trip[0].as<uint8_t>();
                                uint8_t g = trip[1].as<uint8_t>();
                                uint8_t b = trip[2].as<uint8_t>();
                                S.setColor(cIndex, RGBW32(r, g, b, 0));
                            }
                        }
                    }
                    anyChange = true;
                } else {
                    // append new segment
                    int32_t st2 = (st >= 0) ? st : 0;
                    int32_t sp2 = (sp >= 0) ? sp : 0;
                    if (sp2 <= st2) continue;

                    uint8_t newId = strip.getSegmentsNum();
                    strip.setSegment(newId, (uint16_t) st2, (uint16_t) sp2, 1, 0, UINT16_MAX, 0, 1);
                    if (fx >= 0) strip.setMode(newId, (uint8_t) fx);

                    auto &S = strip.getSegment(newId);
                    S.on = on;
                    if (sx >= 0) S.speed = (uint8_t) sx;
                    if (ix >= 0) S.intensity = (uint8_t) ix;
                    if (bri >= 0) S.setOpacity((uint8_t) bri);

                    if (sObj.containsKey("col") && sObj["col"].is<JsonArray>()) {
                        JsonArray colIn = sObj["col"].as<JsonArray>();
                        for (uint8_t cIndex = 0; cIndex < 3 && cIndex < colIn.size(); cIndex++) {
                            JsonArray trip = colIn[cIndex].as<JsonArray>();
                            if (trip.size() == 3) {
                                uint8_t r = trip[0].as<uint8_t>();
                                uint8_t g = trip[1].as<uint8_t>();
                                uint8_t b = trip[2].as<uint8_t>();
                                S.setColor(cIndex, RGBW32(r, g, b, 0));
                            }
                        }
                    }
                    anyChange = true;
                }
            }
        }

        if (anyChange) {
            strip.service(); // immediate refresh
        }

        StaticJsonDocument<256> resp;
        resp["status"] = "ok";
        resp["segCount"] = strip.getSegmentsNum();
        String out;
        serializeJson(resp, out);
        Serial.println(out);
        Serial1.println(out);
        return;
    }

    // 3) Handle “getState” → larger response buffer
    if (strcmp(cmd, "getState") == 0) {
        StaticJsonDocument<RESP_LARGE_CAP> resp;
        JsonArray segArr = resp.createNestedArray("seg");

        uint8_t numSegs = strip.getSegmentsNum();
        for (uint8_t i = 0; i < numSegs; i++) {
            auto &S = strip.getSegment(i);
            JsonObject so = segArr.createNestedObject();
            so["id"]    = i;
            so["on"]    = S.on;
            so["start"] = S.start;
            so["stop"]  = S.stop;
            so["fx"]    = S.mode;
            so["sx"]    = S.speed;
            so["ix"]    = S.intensity;

            JsonArray colOut = so.createNestedArray("col");
            for (uint8_t cidx = 0; cidx < 3; cidx++) {
                CRGB c = S.colors[cidx];
                JsonArray rgb = colOut.createNestedArray();
                rgb.add(c.red); rgb.add(c.green); rgb.add(c.blue);
            }
        }

        String out; serializeJson(resp, out);
        Serial.println("getState Response: ");
        Serial.println(out);
        Serial1.println(out);
        return;
    }

    if (strcmp(cmd, "setTopology") == 0) {
        JsonArray busses = doc["busses"].as<JsonArray>();
        if (busses.isNull() || busses.size() == 0) {
            respondError("busses must be a non-empty array");
            return;
        }

        // Optional global ABL
        if (doc.containsKey("globalMilliAmpsMax")) {
            BusManager::setMilliampsMax((uint16_t) doc["globalMilliAmpsMax"].as<int>());
        }

        // Clear any previous configs
        for (int i = 0; i < WLED_MAX_BUSSES + WLED_MIN_VIRTUAL_BUSSES; i++) {
            if (busConfigs[i]) { delete busConfigs[i]; busConfigs[i] = nullptr; }
        }
        g_busBase.clear();

        // Build new configs with cumulative start offsets
        uint8_t idx = 0;
        uint32_t cumulative = 0;
        for (JsonObject b : busses) {
            if (idx >= WLED_MAX_BUSSES) break;

            uint8_t  pin  = b["pin"]   | 4;
            uint16_t cnt  = b["count"] | 0;
            const char* orderStr = b["colorOrder"] | "GRB";
            if (!cnt) { continue; }

            uint8_t order = COL_ORDER_GRB;
            if      (!strcmp(orderStr, "GRB")) order = COL_ORDER_GRB;
            else if (!strcmp(orderStr, "BRG")) order = COL_ORDER_BRG;
            else if (!strcmp(orderStr, "RGB")) order = COL_ORDER_RGB;
            // add more if needed

            uint8_t pins[] = { pin };
            busConfigs[idx] = new BusConfig(
              TYPE_WS2812_RGB,
              pins,
              cumulative,   // startOffset (global pixel index)
              cnt,          // count
              order
            );

            if (b.containsKey("milliAmpsPerLed")) busConfigs[idx]->milliAmpsPerLed = (uint16_t)b["milliAmpsPerLed"].as<int>();
            if (b.containsKey("milliAmpsMax"))    busConfigs[idx]->milliAmpsMax    = (uint16_t)b["milliAmpsMax"].as<int>();

            g_busBase.push_back(cumulative);
            cumulative += cnt;
            idx++;
        }

        doInitBusses = true;             // ask WLED to rebuild hardware busses
        g_topologyConfigured = true;

        respondOk("busCount", (int)idx);
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
