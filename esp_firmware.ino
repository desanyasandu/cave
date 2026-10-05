/* Edge Impulse Arduino examples
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// These sketches are tested with 2.0.4 ESP32 Arduino Core
// https://github.com/espressif/arduino-esp32/releases/tag/2.0.4

/* Includes ---------------------------------------------------------------- */
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <DatasetforArm_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include <ctype.h>

#include "esp_camera.h"
#include <WebServer.h>

// Select camera model - find more camera models in camera_pins.h file here
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h

//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#if defined(CAMERA_MODEL_ESP_EYE)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    4
#define SIOD_GPIO_NUM    18
#define SIOC_GPIO_NUM    23

#define Y9_GPIO_NUM      36
#define Y8_GPIO_NUM      37
#define Y7_GPIO_NUM      38
#define Y6_GPIO_NUM      39
#define Y5_GPIO_NUM      35
#define Y4_GPIO_NUM      14
#define Y3_GPIO_NUM      13
#define Y2_GPIO_NUM      34
#define VSYNC_GPIO_NUM   5
#define HREF_GPIO_NUM    27
#define PCLK_GPIO_NUM    25

#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#else
#error "Camera model not selected"
#endif

#if defined(CAMERA_MODEL_AI_THINKER)
const int SEARCH_LED_PIN = 4;  // Onboard flash LED (AI Thinker ESP32-CAM)
const uint8_t SEARCH_LED_ON = HIGH;
const uint8_t SEARCH_LED_OFF = LOW;
#else
const int SEARCH_LED_PIN = -1;
const uint8_t SEARCH_LED_ON = LOW;
const uint8_t SEARCH_LED_OFF = LOW;
#endif

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

// WiFi credentials
const char* ssid = "swifi";
const char* password = "12345678";

// WebSocket server port
#define WEBSOCKET_PORT 81

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false;
static bool is_initialised = false;
uint8_t *snapshot_buf;
WebSocketsServer webSocket = WebSocketsServer(WEBSOCKET_PORT);
WebServer server(80);

// ===== Autonomous Mission Settings =====
const float MISSION_MATCH_THRESHOLD = 0.50f;
const int MISSION_CONFIRM_FRAMES = 1;
const uint16_t DEFAULT_CAR_PORT = 80;
const unsigned long CAR_COMMAND_TIMEOUT_MS = 2500;
const unsigned long MISSION_MIN_SEARCH_MS = 0;  // trigger pick immediately when target is detected

bool missionActive = false;
String missionItem = "";
String missionItemNorm = "";
String missionCarIp = "";
uint16_t missionCarPort = DEFAULT_CAR_PORT;
uint8_t missionClientNum = 255;
int missionConsecutiveMatches = 0;
unsigned long missionStartedAtMs = 0;
bool searchLedState = false;

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) ;
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void sendPredictionsToWebSocket(ei_impulse_result_t &result);
void processMissionDetection(ei_impulse_result_t &result);
void handleArmCommand(const String& command, uint8_t clientNum);
bool forwardPickToCar(const String& itemName, const String& carIp, uint16_t carPort, String& responseLine);
bool sendCommandToCar(const String& command, const String& carIp, uint16_t carPort, String& responseLine, unsigned long timeoutMs);
void clearMissionState();
String normalizeLabel(const String& value);
float getTargetConfidence(ei_impulse_result_t &result, const String& normalizedTarget, String& matchedLabel);
void sendStatusToClient(uint8_t clientNum, const String& status, const String& item, const String& detail);
void cancelActiveMissionOnCar();
void setupSearchLed();
void setSearchLed(bool enabled);
void handleRoot();
void handleCSS();
void handleJS();
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

/**
* @brief      Arduino setup function
*/
void setup()
{
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Edge Impulse Inferencing Demo");
    setupSearchLed();
    
    // Connect to WiFi
    Serial.println();
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // Start WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("WebSocket server started");
    Serial.print("WebSocket URL: ws://");
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.println(WEBSOCKET_PORT);
    
    // Start HTTP server
    server.on("/", handleRoot);
    server.begin();
    Serial.println("HTTP Server started");
    Serial.print("Access at: http://");
    Serial.print(WiFi.localIP());
    Serial.println("/");
    
    if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
    }
    else {
        ei_printf("Camera initialized\r\n");
    }

    ei_printf("\nStarting continious inference in 2 seconds...\n");
    ei_sleep(2000);
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop()
{
    // Process WebSocket events every loop so incoming connections and messages are handled.
    webSocket.loop();
    
    // Process HTTP server requests
    server.handleClient();

    // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    // check if allocation was successful
    if(snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        free(snapshot_buf);
        return;
    }

    // print the predictions
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }
#else
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
    ei_printf("Visual anomalies:\r\n");
    for (uint32_t i = 0; i < result.visual_ad_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.visual_ad_grid_cells[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }
#endif

    // Send predictions to WebSocket clients
    sendPredictionsToWebSocket(result);
    processMissionDetection(result);

    free(snapshot_buf);

}

/**
 * @brief   WebSocket event handler
 */
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            if (missionActive && missionClientNum == num) {
                Serial.println("Active mission client disconnected. Cancelling mission.");
                cancelActiveMissionOnCar();
            }
            break;
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket.remoteIP(num);
                Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
                webSocket.sendTXT(num, "{\"status\":\"connected\"}");
            }
            break;
        case WStype_TEXT:
            {
                String command;
                command.reserve(length);
                for (size_t i = 0; i < length; i++) {
                    command += (char)payload[i];
                }
                command.trim();
                Serial.printf("[%u] Text: %s\n", num, command.c_str());
                handleArmCommand(command, num);
            }
            break;
        case WStype_BIN:
            Serial.printf("[%u] Binary data length: %u\n", num, length);
            break;
        case WStype_PING:
            Serial.printf("[%u] PING\n", num);
            break;
        case WStype_PONG:
            Serial.printf("[%u] PONG\n", num);
            break;
    }
}

/**
 * @brief   Handle arm control commands from frontend
 */
void handleArmCommand(const String& command, uint8_t clientNum) {
    Serial.println("Command received: " + command);

    if (command.startsWith("mission_start|")) {
        int p1 = command.indexOf('|');
        int p2 = command.indexOf('|', p1 + 1);
        int p3 = command.indexOf('|', p2 + 1);

        if (p1 < 0 || p2 < 0 || p3 < 0) {
            sendStatusToClient(clientNum, "pick_failed", "", "invalid_request_format");
            return;
        }

        String itemName = command.substring(p1 + 1, p2);
        String carIp = command.substring(p2 + 1, p3);
        String portStr = command.substring(p3 + 1);
        itemName.trim();
        carIp.trim();
        portStr.trim();

        uint16_t carPort = (uint16_t)portStr.toInt();
        if (carPort == 0) {
            carPort = DEFAULT_CAR_PORT;
        }

        if (itemName.length() == 0 || carIp.length() == 0) {
            sendStatusToClient(clientNum, "pick_failed", itemName, "missing_item_or_car_ip");
            return;
        }

        String itemNorm = normalizeLabel(itemName);
        if (itemNorm.length() == 0) {
            sendStatusToClient(clientNum, "pick_failed", itemName, "invalid_item_name");
            return;
        }

        if (missionActive) {
            String previousItem = missionItem;
            cancelActiveMissionOnCar();
            sendStatusToClient(clientNum, "mission_replaced", previousItem, "previous_active_mission_cancelled");
        }

        String responseLine;
        bool ok = sendCommandToCar("mission_start|" + itemName, carIp, carPort, responseLine, CAR_COMMAND_TIMEOUT_MS);
        if (!ok) {
            sendStatusToClient(clientNum, "pick_failed", itemName, responseLine);
            return;
        }

        missionActive = true;
        missionItem = itemName;
        missionItemNorm = itemNorm;
        missionCarIp = carIp;
        missionCarPort = carPort;
        missionClientNum = clientNum;
        missionConsecutiveMatches = 0;
        missionStartedAtMs = millis();
        setSearchLed(true);

        sendStatusToClient(clientNum, "mission_started", itemName, responseLine);
        return;
    }

    if (command.equalsIgnoreCase("mission_cancel")) {
        if (!missionActive) {
            sendStatusToClient(clientNum, "mission_cancelled", "", "no_active_mission");
            return;
        }

        String responseLine;
        sendCommandToCar("mission_cancel", missionCarIp, missionCarPort, responseLine, CAR_COMMAND_TIMEOUT_MS);
        clearMissionState();
        sendStatusToClient(clientNum, "mission_cancelled", "", responseLine);
        return;
    }

    if (command.startsWith("pick_request|")) {
        int p1 = command.indexOf('|');
        int p2 = command.indexOf('|', p1 + 1);
        int p3 = command.indexOf('|', p2 + 1);

        if (p1 < 0 || p2 < 0 || p3 < 0) {
            sendStatusToClient(clientNum, "pick_failed", "", "invalid_request_format");
            return;
        }

        String itemName = command.substring(p1 + 1, p2);
        String carIp = command.substring(p2 + 1, p3);
        String portStr = command.substring(p3 + 1);
        itemName.trim();
        carIp.trim();
        portStr.trim();

        uint16_t carPort = (uint16_t)portStr.toInt();
        if (carPort == 0) {
            carPort = 80;
        }

        if (itemName.length() == 0 || carIp.length() == 0) {
            sendStatusToClient(clientNum, "pick_failed", itemName, "missing_item_or_car_ip");
            return;
        }

        sendStatusToClient(clientNum, "pick_triggered", itemName, "forwarding_to_car");

        String responseLine;
        clearMissionState();
        bool ok = forwardPickToCar(itemName, carIp, carPort, responseLine);
        if (ok) {
            sendStatusToClient(clientNum, "pick_forwarded", itemName, responseLine);
        } else {
            sendStatusToClient(clientNum, "pick_failed", itemName, responseLine);
        }
        return;
    }
    else {
        sendStatusToClient(clientNum, "ignored_command", "", "use_mission_start_or_pick_request");
    }
}

void clearMissionState() {
    missionActive = false;
    missionItem = "";
    missionItemNorm = "";
    missionCarIp = "";
    missionCarPort = DEFAULT_CAR_PORT;
    missionClientNum = 255;
    missionConsecutiveMatches = 0;
    missionStartedAtMs = 0;
    setSearchLed(false);
}

void setupSearchLed() {
    if (SEARCH_LED_PIN < 0) {
        return;
    }

    pinMode(SEARCH_LED_PIN, OUTPUT);
    digitalWrite(SEARCH_LED_PIN, SEARCH_LED_OFF);
    searchLedState = false;
}

void setSearchLed(bool enabled) {
    if (SEARCH_LED_PIN < 0) {
        return;
    }

    if (enabled == searchLedState) {
        return;
    }

    digitalWrite(SEARCH_LED_PIN, enabled ? SEARCH_LED_ON : SEARCH_LED_OFF);
    searchLedState = enabled;
}

void cancelActiveMissionOnCar() {
    if (!missionActive) {
        return;
    }

    String responseLine;
    bool ok = false;

    if (missionCarIp.length() > 0) {
        ok = sendCommandToCar("mission_cancel", missionCarIp, missionCarPort, responseLine, CAR_COMMAND_TIMEOUT_MS);
    } else {
        responseLine = "no_car_target_in_mission_state";
    }

    if (ok) {
        Serial.println("Previous mission cancelled on car.");
    } else {
        Serial.println("Mission cancel command did not get ACK. Clearing local mission state anyway.");
    }

    clearMissionState();
}

String normalizeLabel(const String& value) {
    String out;
    out.reserve(value.length());

    for (int i = 0; i < value.length(); i++) {
        unsigned char c = (unsigned char)value.charAt(i);
        if (isalnum(c)) {
            out += (char)tolower(c);
        }
    }

    return out;
}

float getTargetConfidence(ei_impulse_result_t &result, const String& normalizedTarget, String& matchedLabel) {
    matchedLabel = "";
    if (normalizedTarget.length() == 0) {
        return 0.0f;
    }

    float best = 0.0f;

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value <= 0.0f) {
            continue;
        }

        String label = String(bb.label);
        if (normalizeLabel(label) == normalizedTarget && bb.value > best) {
            best = bb.value;
            matchedLabel = label;
        }
    }
#else
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        String label = String(ei_classifier_inferencing_categories[i]);
        float value = result.classification[i].value;
        if (normalizeLabel(label) == normalizedTarget && value > best) {
            best = value;
            matchedLabel = label;
        }
    }
#endif

    return best;
}

void processMissionDetection(ei_impulse_result_t &result) {
    if (!missionActive) {
        return;
    }

    if (millis() - missionStartedAtMs < MISSION_MIN_SEARCH_MS) {
        return;
    }

    String matchedLabel;
    float confidence = getTargetConfidence(result, missionItemNorm, matchedLabel);

    if (confidence >= MISSION_MATCH_THRESHOLD) {
        missionConsecutiveMatches++;
        if (missionConsecutiveMatches == 1) {
            String detail = matchedLabel + ":" + String(confidence, 3);
            sendStatusToClient(missionClientNum, "mission_target_seen", missionItem, detail);
        }
    } else {
        missionConsecutiveMatches = 0;
        return;
    }

    if (missionConsecutiveMatches < MISSION_CONFIRM_FRAMES) {
        return;
    }

    sendStatusToClient(missionClientNum, "pick_triggered", missionItem, "target_confirmed");

    String responseLine;
    bool ok = sendCommandToCar("pick_now|" + missionItem, missionCarIp, missionCarPort, responseLine, CAR_COMMAND_TIMEOUT_MS);
    if (ok) {
        sendStatusToClient(missionClientNum, "pick_forwarded", missionItem, responseLine);
    } else {
        sendStatusToClient(missionClientNum, "pick_failed", missionItem, responseLine);
    }

    clearMissionState();
}

bool sendCommandToCar(const String& command, const String& carIp, uint16_t carPort, String& responseLine, unsigned long timeoutMs) {
    WiFiClient carClient;
    if (WiFi.status() != WL_CONNECTED) {
        responseLine = "esp_wifi_not_connected";
        return false;
    }

    bool connected = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        if (carClient.connect(carIp.c_str(), carPort)) {
            connected = true;
            break;
        }
        delay(80);
    }

    if (!connected) {
        responseLine = "connect_failed_" + carIp + ":" + String(carPort);
        return false;
    }

    carClient.print(command);
    carClient.print('\n');

    unsigned long start = millis();
    responseLine = "";
    while (millis() - start < timeoutMs) {
        if (carClient.available()) {
            responseLine = carClient.readStringUntil('\n');
            responseLine.trim();
            break;
        }
        delay(10);
    }

    carClient.stop();

    if (responseLine.length() == 0) {
        responseLine = "no_reply_" + carIp + ":" + String(carPort);
        return false;
    }

    return responseLine.startsWith("OK:");
}

bool forwardPickToCar(const String& itemName, const String& carIp, uint16_t carPort, String& responseLine) {
    return sendCommandToCar(itemName, carIp, carPort, responseLine, CAR_COMMAND_TIMEOUT_MS);
}

void sendStatusToClient(uint8_t clientNum, const String& status, const String& item, const String& detail) {
    String json = "{\"status\":\"" + status + "\"";
    if (item.length() > 0) {
        json += ",\"item\":\"" + item + "\"";
    }
    if (detail.length() > 0) {
        json += ",\"detail\":\"" + detail + "\"";
    }
    json += "}";
    if (clientNum == 255) {
        webSocket.broadcastTXT(json);
    } else {
        webSocket.sendTXT(clientNum, json);
    }
}

/**
 * @brief   Send prediction results to WebSocket clients
 */
void sendPredictionsToWebSocket(ei_impulse_result_t &result) {
    String json = "{";
    json += "\"timing\":{";
    json += "\"dsp\":" + String(result.timing.dsp);
    json += ",\"classification\":" + String(result.timing.classification);
    json += ",\"anomaly\":" + String(result.timing.anomaly);
    json += "},";

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    json += "\"type\":\"object_detection\",";
    json += "\"bounding_boxes\":[";
    bool firstBox = true;
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) continue;
        if (!firstBox) json += ",";
        firstBox = false;
        json += "{";
        json += "\"label\":\"" + String(bb.label) + "\",";
        json += "\"value\":" + String(bb.value, 6);
        json += ",\"x\":" + String(bb.x);
        json += ",\"y\":" + String(bb.y);
        json += ",\"width\":" + String(bb.width);
        json += ",\"height\":" + String(bb.height);
        json += "}";
    }
    json += "]";
#else
    json += "\"type\":\"classification\",";
    json += "\"predictions\":[";
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (i > 0) json += ",";
        json += "{";
        json += "\"label\":\"" + String(ei_classifier_inferencing_categories[i]) + "\",";
        json += "\"value\":" + String(result.classification[i].value, 5);
        json += "}";
    }
    json += "]";
#endif

#if EI_CLASSIFIER_HAS_ANOMALY
    json += ",\"anomaly\":" + String(result.anomaly, 3);
#endif

    json += "}";

    webSocket.broadcastTXT(json);
}

/**
 * @brief   Handle HTTP root request - serve control panel
 */
void handleRoot() {
    String html = R"html(<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Picker Control Panel</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; display: flex; justify-content: center; align-items: center; padding: 20px; }
        .container { display: grid; grid-template-columns: 1fr 1fr; gap: 30px; max-width: 1200px; width: 100%; }
        .panel { background: white; border-radius: 15px; padding: 30px; box-shadow: 0 20px 60px rgba(0,0,0,0.3); }
        h1 { color: #333; margin-bottom: 10px; font-size: 28px; }
        .status { display: flex; align-items: center; gap: 10px; margin-bottom: 20px; padding: 15px; border-radius: 10px; background: #f5f5f5; }
        .status-dot { width: 12px; height: 12px; border-radius: 50%; background: #ff4444; animation: blink 1s infinite; }
        .status-dot.connected { background: #00cc00; animation: none; }
        @keyframes blink { 0%, 49% { opacity: 1; } 50%, 100% { opacity: 0.3; } }
        h2 { color: #667eea; margin: 20px 0 15px 0; font-size: 18px; }
        .button-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; margin-bottom: 20px; }
        button { padding: 15px; border: none; border-radius: 10px; font-size: 14px; font-weight: 600; cursor: pointer; transition: all 0.3s ease; color: white; }
        .btn-direction { background: #667eea; }
        .btn-direction:hover { background: #5568d3; transform: translateY(-2px); box-shadow: 0 5px 15px rgba(102, 126, 234, 0.4); }
        .btn-grab { background: #ff6b6b; grid-column: 2; padding: 20px; font-size: 16px; }
        .btn-grab:hover { background: #ff5252; box-shadow: 0 5px 15px rgba(255, 107, 107, 0.4); }
        .auto-controls { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 20px; }
        .btn-auto { background: #51cf66; padding: 15px; font-size: 14px; color: white; border: none; border-radius: 10px; cursor: pointer; }
        .btn-auto:hover { background: #40c057; box-shadow: 0 5px 15px rgba(81, 207, 102, 0.4); }
        .predictions { background: #f9f9f9; padding: 20px; border-radius: 10px; max-height: 400px; overflow-y: auto; }
        .prediction-item { padding: 12px; margin-bottom: 10px; background: white; border-left: 4px solid #667eea; border-radius: 5px; display: flex; justify-content: space-between; align-items: center; }
        .cart-count { font-size: 24px; font-weight: 700; color: #ff6b6b; text-align: center; margin: 15px 0; }
        @media (max-width: 768px) { .container { grid-template-columns: 1fr; } }
    </style>
</head>
<body>
    <div class="container">
        <div class="panel">
            <h1>🤖 Robot Picker</h1>
            <div class="status">
                <div class="status-dot" id="statusDot"></div>
                <span id="statusText">Connecting...</span>
            </div>
            <h2>Manual Controls</h2>
            <div class="button-grid">
                <button class="btn-direction" onclick="sendCommand('forward')">⬆ UP</button>
                <button class="btn-direction" onclick="sendCommand('left')">⬅ LEFT</button>
                <button class="btn-grab" onclick="sendCommand('grab')">GRAB</button>
                <button class="btn-direction" onclick="sendCommand('right')">RIGHT ➡</button>
                <button class="btn-direction" onclick="sendCommand('backward')">⬇ DOWN</button>
            </div>
            <h2>Auto Grab</h2>
            <div class="auto-controls">
                <button class="btn-auto" onclick="autoGrab('left')">📍 Grab LEFT</button>
                <button class="btn-auto" onclick="autoGrab('right')">📍 Grab RIGHT</button>
            </div>
            <div style="background: #e8f5e9; border-left: 4px solid #51cf66; padding: 15px; border-radius: 5px; margin: 15px 0; font-size: 13px;">
                <strong>📋 Instructions:</strong><br> 1. Camera detects items on racks<br> 2. Use "Grab LEFT/RIGHT" for auto pick<br> 3. Manual buttons for precise control
            </div>
            <div style="background: #fff3cd; border-left: 4px solid #ffc107; padding: 15px; border-radius: 5px; margin: 15px 0;">
                <strong>🛒 Items in Cart:</strong>
                <div class="cart-count" id="cartCount">0</div>
            </div>
        </div>
        <div class="panel">
            <h1>📊 Detection Results</h1>
            <div class="status">
                <div class="status-dot" id="detectionDot"></div>
                <span id="detectionText">Waiting for predictions...</span>
            </div>
            <h2>Detected Items</h2>
            <div class="predictions" id="predictions" style="color: #999; text-align: center; padding: 40px;">Awaiting camera input...</div>
        </div>
    </div>
    <script>
        let ws;
        let cartItems = 0;
        function connectWebSocket() {
            ws = new WebSocket('ws://' + window.location.hostname + ':81');
            ws.onopen = () => { updateStatus(true); };
            ws.onmessage = (e) => { try { const data = JSON.parse(e.data); updatePredictions(data); } catch(e) {} };
            ws.onerror = () => { updateStatus(false); };
            ws.onclose = () => { updateStatus(false); setTimeout(connectWebSocket, 3000); };
        }
        function updateStatus(c) { 
            const d = document.getElementById('statusDot');
            if(c) { d.classList.add('connected'); document.getElementById('statusText').textContent = 'Connected'; }
            else { d.classList.remove('connected'); document.getElementById('statusText').textContent = 'Disconnected'; }
        }
        function sendCommand(cmd) {
            if(ws && ws.readyState === 1) ws.send(cmd);
        }
        function autoGrab(side) {
            if(ws && ws.readyState === 1) { ws.send('auto_grab_' + side); cartItems++; document.getElementById('cartCount').textContent = cartItems; }
        }
        function updatePredictions(data) {
            if(data.predictions) {
                let html = '';
                data.predictions.forEach(p => { html += '<div class="prediction-item"><span>' + p.label + '</span><span>' + Math.round(p.value*100) + '%</span></div>'; });
                document.getElementById('predictions').innerHTML = html ? html : 'No predictions';
                document.getElementById('detectionDot').classList.add('connected');
            }
        }
        window.addEventListener('load', () => { connectWebSocket(); });
        document.addEventListener('keydown', (e) => {
            switch(e.key) { case 'ArrowUp': sendCommand('forward'); break; case 'ArrowDown': sendCommand('backward'); break; case 'ArrowLeft': sendCommand('left'); break; case 'ArrowRight': sendCommand('right'); break; case ' ': e.preventDefault(); sendCommand('grab'); break; }
        });
    </script>
</body>
</html>
)html";
    server.send(200, "text/html", html);
}

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {

    if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x\n", err);
      return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1); // flip it back
      s->set_brightness(s, 1); // up the brightness just a bit
      s->set_saturation(s, 0); // lower the saturation
    }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_awb_gain(s, 1);
#endif

    is_initialised = true;
    return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {

    //deinitialize the camera
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK)
    {
        ei_printf("Camera deinit failed\n");
        return;
    }

    is_initialised = false;
    return;
}


/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

   bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

   esp_camera_fb_return(fb);

   if(!converted){
       ei_printf("Conversion failed\n");
       return false;
   }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }


    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        // Swap BGR to RGB here
        // due to https://github.com/espressif/esp32-camera/issues/379
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // and done!
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
