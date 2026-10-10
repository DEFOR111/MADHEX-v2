#include <Arduino.h>
#include <Preferences.h>
#include "calibration_store.h"

const char CALIB_NAMESPACE[] = "calib";
const char CALIB_KEY_DATA[] = "data";
const char CALIB_KEY_VERSION[] = "version";
const uint8_t CALIB_VERSION = 1;

bool calibrationSave(const Calibration &c) {
    Preferences prefs;
    if (!prefs.begin(CALIB_NAMESPACE, false)) {
        return false;
    }
    size_t written = prefs.putBytes(CALIB_KEY_DATA, &c, sizeof(c));
    bool versionOk = prefs.putUChar(CALIB_KEY_VERSION, CALIB_VERSION) == 1;
    prefs.end();
    return written == sizeof(c) && versionOk;
}

bool calibrationLoad(Calibration &c) {
    c = makeCalibration();
    Preferences prefs;
    if (!prefs.begin(CALIB_NAMESPACE, true)) {
        return false;
    }
    bool ok = false;
    if (prefs.getUChar(CALIB_KEY_VERSION, 0) == CALIB_VERSION
        && prefs.getBytesLength(CALIB_KEY_DATA) == sizeof(Calibration)) {
        Calibration loaded;
        prefs.getBytes(CALIB_KEY_DATA, &loaded, sizeof(loaded));
        if (calibrationIsValid(loaded)) {
            c = loaded;
            ok = true;
        }
    }
    prefs.end();
    return ok;
}
