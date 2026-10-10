#include <stdio.h>
#include <string.h>
#include "web_status.h"

static void add(char out[], int size, int &n, bool &ok, const char *text) {
    int length = (int)strlen(text);
    if (!ok || n + length >= size) {
        ok = false;
        return;
    }
    memcpy(out + n, text, length + 1);
    n += length;
}

int webStatusJson(const WebStatus &s, char out[], int size) {
    char part[128];
    int n = 0;
    bool ok = (size > 0);
    if (ok) {
        out[0] = '\0';
    }

    snprintf(part, sizeof(part), "{\"type\":\"status\",\"state\":\"%s\",\"gait\":\"%s\",\"servos\":\"%s\",",
             s.state, s.gait, s.servos);
    add(out, size, n, ok, part);
    snprintf(part, sizeof(part), "\"height\":%.0f,\"height_min\":%.0f,\"height_max\":%.0f,\"level\":%s,",
             s.heightMm, s.heightMinMm, s.heightMaxMm, s.levelOn ? "true" : "false");
    add(out, size, n, ok, part);

    if (s.batteryValid) {
        snprintf(part, sizeof(part), "\"batt_v\":%.2f,\"batt_state\":\"%s\",", s.batteryV, s.batteryState);
    } else {
        snprintf(part, sizeof(part), "\"batt_v\":null,\"batt_state\":null,");
    }
    add(out, size, n, ok, part);
    if (s.currentValid) {
        snprintf(part, sizeof(part), "\"current_a\":%.3f,", s.currentA);
    } else {
        snprintf(part, sizeof(part), "\"current_a\":null,");
    }
    add(out, size, n, ok, part);
    if (s.imuValid) {
        snprintf(part, sizeof(part), "\"roll\":%.1f,\"pitch\":%.1f,", s.rollDeg, s.pitchDeg);
    } else {
        snprintf(part, sizeof(part), "\"roll\":null,\"pitch\":null,");
    }
    add(out, size, n, ok, part);

    snprintf(part, sizeof(part), "\"led\":{\"color\":\"#%02x%02x%02x\",\"blink_ms\":%lu,\"flash\":%s},",
             s.led.color.r, s.led.color.g, s.led.color.b, s.led.blinkMs, s.led.whiteFlash ? "true" : "false");
    add(out, size, n, ok, part);
    snprintf(part, sizeof(part), "\"loop\":{\"ups\":%d,\"max_loop_us\":%lu,\"max_gap_ms\":%.1f}}",
             s.loop.updatesPerSecond, s.loop.maxLoopUs, s.loop.maxGapUs / 1000.0f);
    add(out, size, n, ok, part);

    return ok ? n : 0;
}

int webReplyJson(const char *text, char out[], int size) {
    int n = 0;
    bool ok = (size > 0);
    if (ok) {
        out[0] = '\0';
    }
    add(out, size, n, ok, "{\"type\":\"reply\",\"text\":\"");
    int end = (int)strlen(text);
    while (end > 0 && (text[end - 1] == '\n' || text[end - 1] == '\r')) {
        end--;
    }
    for (int i = 0; i < end; i++) {
        char c = text[i];
        char piece[3] = { c, '\0', '\0' };
        if (c == '"' || c == '\\') {
            piece[0] = '\\';
            piece[1] = c;
        } else if (c == '\n') {
            piece[0] = '\\';
            piece[1] = 'n';
        } else if ((unsigned char)c < 0x20) {
            continue;
        }
        add(out, size, n, ok, piece);
    }
    add(out, size, n, ok, "\"}");
    return ok ? n : 0;
}

bool webLinkLost(bool driving, unsigned long msSinceLastMessage, unsigned long timeoutMs) {
    return driving && msSinceLastMessage > timeoutMs;
}

bool webIsWalkCommand(const char *line) {
    while (line[0] == ' ') {
        line++;
    }
    char word[8];
    int i = 0;
    while (line[i] != '\0' && line[i] != ' ' && i < 7) {
        word[i] = line[i];
        i++;
    }
    word[i] = '\0';
    if (line[i] != '\0' && line[i] != ' ') {
        return false;
    }
    return strcmp(word, "walk") == 0 || strcmp(word, "w") == 0 || strcmp(word, "s") == 0
        || strcmp(word, "a") == 0 || strcmp(word, "d") == 0 || strcmp(word, "q") == 0
        || strcmp(word, "e") == 0;
}
