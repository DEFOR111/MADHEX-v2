#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include "config.h"
#include "web.h"
#include "commands.h"
#include "power_monitor_hw.h"
#include "imu_hw.h"
#include "servo_output_hw.h"

enum WebEventKind { WEB_EVENT_CONNECT, WEB_EVENT_DISCONNECT, WEB_EVENT_TEXT };

struct WebEvent {
    WebEventKind kind;
    uint32_t client;
    char text[64];
};

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");
static QueueHandle_t events = NULL;
static bool started = false;

static bool driving = false;
static uint32_t drivingClient = 0;
static unsigned long lastMessageMs = 0;
static unsigned long lastStatusMs = 0;
static unsigned long lastCleanupMs = 0;

class ReplyBuffer : public Print {
public:
    char text[256];
    int length;

    ReplyBuffer() {
        length = 0;
        text[0] = '\0';
    }
    size_t write(uint8_t c) override {
        if (length < (int)sizeof(text) - 1) {
            text[length] = (char)c;
            length++;
            text[length] = '\0';
        }
        return 1;
    }
};

static void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
                             void *arg, uint8_t *data, size_t len) {
    WebEvent e;
    e.client = client->id();
    e.text[0] = '\0';
    if (type == WS_EVT_CONNECT) {
        e.kind = WEB_EVENT_CONNECT;
    } else if (type == WS_EVT_DISCONNECT) {
        e.kind = WEB_EVENT_DISCONNECT;
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (!info->final || info->index != 0 || info->len != len || info->opcode != WS_TEXT
            || len >= sizeof(e.text)) {
            return;
        }
        e.kind = WEB_EVENT_TEXT;
        memcpy(e.text, data, len);
        e.text[len] = '\0';
    } else {
        return;
    }
    xQueueSend(events, &e, 0);
}

static void handleRoot(AsyncWebServerRequest *request) {
    if (LittleFS.exists("/index.html")) {
        request->send(LittleFS, "/index.html", "text/html");
    } else {
        request->send(404, "text/plain", "index.html is not in the flash: run 'pio run -t uploadfs'");
    }
}

void webBegin() {
    events = xQueueCreate(16, sizeof(WebEvent));
    WiFi.mode(WIFI_AP);
    bool apOk = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
    bool fsOk = LittleFS.begin(false);
    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);
    server.on("/", HTTP_GET, handleRoot);
    server.begin();
    started = apOk && events != NULL;

    char line[160];
    int n;
    if (started) {
        n = snprintf(line, sizeof(line), "web: Wi-Fi '%s' ready, open http://%s%s\r\n", WIFI_AP_SSID,
                     WiFi.softAPIP().toString().c_str(),
                     fsOk ? "" : " (LittleFS missing: run 'pio run -t uploadfs')");
    } else {
        n = snprintf(line, sizeof(line), "web: Wi-Fi access point failed, no web control\r\n");
    }
    Serial.write((const uint8_t *)line, n);
}

static void stopDriving(Controller &ctrl, const char *reason) {
    driving = false;
    ControlReply r = controllerStop(ctrl);
    char line[120];
    int n = snprintf(line, sizeof(line), "web: %s -> %s\r\n", reason, r.message);
    Serial.write((const uint8_t *)line, n);
}

static void sendStatus(const Controller &ctrl, const Safety &safety, const Leveling &leveling,
                       const LoopStats &loop) {
    PowerReading power = powerLatest();
    ImuReading imu = imuLatest();

    LedInputs in;
    in.state = ctrl.state;
    in.safetyFault = (safety.fault != SAFETY_FAULT_NONE);
    in.batteryValid = power.valid;
    in.battery = power.state;
    in.servoOutputOff = (servoOutputMode() == SERVO_OUTPUT_OFF);

    WebStatus s;
    s.state = stateName(ctrl.state);
    s.gait = gaitName(ctrl.gait.type);
    s.servos = servoOutputModeName(servoOutputMode());
    s.heightMm = controllerHeightMm(ctrl);
    s.heightMinMm = STAND_HEIGHT_MIN_MM;
    s.heightMaxMm = STAND_HEIGHT_MAX_MM;
    s.levelOn = leveling.enabled;
    s.batteryValid = power.valid;
    s.batteryV = power.voltageMv / 1000.0f;
    s.batteryState = batteryStateName(power.state);
    s.currentValid = power.valid && power.fromIna226;
    s.currentA = power.currentMa / 1000.0f;
    s.imuValid = imuPresent() && imu.valid && imu.tiltValid;
    s.rollDeg = imu.rollDeg;
    s.pitchDeg = imu.pitchDeg;
    s.led = statusLedPattern(in);
    s.loop = loop;

    char json[512];
    int n = webStatusJson(s, json, sizeof(json));
    if (n > 0) {
        ws.textAll(json, n);
    }
}

void webUpdate(Controller &ctrl, Calibration &calib, Safety &safety, Leveling &leveling, const LoopStats &loop) {
    if (!started) {
        return;
    }
    unsigned long now = millis();
    char line[96];
    int n;

    WebEvent e;
    while (xQueueReceive(events, &e, 0) == pdTRUE) {
        if (e.kind == WEB_EVENT_CONNECT) {
            n = snprintf(line, sizeof(line), "web: phone %lu connected\r\n", (unsigned long)e.client);
            Serial.write((const uint8_t *)line, n);
        } else if (e.kind == WEB_EVENT_DISCONNECT) {
            n = snprintf(line, sizeof(line), "web: phone %lu disconnected\r\n", (unsigned long)e.client);
            Serial.write((const uint8_t *)line, n);
            if (driving && e.client == drivingClient) {
                stopDriving(ctrl, "the driving phone disconnected");
            }
        } else {
            if (!driving || e.client == drivingClient) {
                lastMessageMs = now;
            }
            ReplyBuffer reply;
            commandsRunLine(e.text, reply, ctrl, calib, safety, leveling);
            if (webIsWalkCommand(e.text) && ctrl.state == STATE_WALK) {
                driving = true;
                drivingClient = e.client;
                lastMessageMs = now;
            }
            if (reply.length == 0) {
                reply.print("(the answer went to Serial)");
            }
            char json[384];
            int length = webReplyJson(reply.text, json, sizeof(json));
            if (length > 0) {
                ws.text(e.client, json, length);
            }
        }
    }

    if (driving && ctrl.state != STATE_WALK) {
        driving = false;
    }
    if (webLinkLost(driving, now - lastMessageMs, WEB_LINK_TIMEOUT_MS)) {
        stopDriving(ctrl, "no command from the driving phone (link lost)");
    }

    if (now - lastStatusMs >= WEB_STATUS_PERIOD_MS) {
        lastStatusMs = now;
        if (ws.count() > 0) {
            sendStatus(ctrl, safety, leveling, loop);
        }
    }
    if (now - lastCleanupMs >= 1000) {
        lastCleanupMs = now;
        ws.cleanupClients(WEB_MAX_CLIENTS);
    }
}
