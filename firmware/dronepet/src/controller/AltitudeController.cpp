#include "AltitudeController.h"

#include <algorithm>

namespace {

constexpr float CONTROL_RATE_HZ = 100.0f;
constexpr float ALTITUDE_FILTER_HZ = 10.0f;

// Approximate hover throttle from our recent tests.
constexpr float HOVER_THROTTLE = 0.32f;

constexpr float ALTITUDE_KP = 0.40f;
constexpr float ALTITUDE_KI = 0.10f;
constexpr float ALTITUDE_KD = 0.08f;

// Keep altitude corrections limited while the controller is still being tuned.
constexpr float MAX_CORRECTION = 0.08f;
constexpr float MAX_I_CORRECTION = 0.04f;

}

namespace DronePet {

void AltitudeController::begin() {
    _altitudeFilter.init(CONTROL_RATE_HZ, ALTITUDE_FILTER_HZ);
    _filterInitialized = false;

    _pid.rate = CONTROL_RATE_HZ;
    _pid.Kp = ALTITUDE_KP;
    _pid.Ki = ALTITUDE_KI;
    _pid.Kd = ALTITUDE_KD;
    _pid.Kf = 0.0f;

    _pid.oLimitLow = -MAX_CORRECTION;
    _pid.oLimitHigh = MAX_CORRECTION;
    _pid.iLimitLow = -MAX_I_CORRECTION;
    _pid.iLimitHigh = MAX_I_CORRECTION;

    _pid.begin();
}

void AltitudeController::reset() {
    _pid.resetIterm();
    _filterInitialized = false;
    _debug = {};
}

float AltitudeController::update(float targetAltitudeM, float altitudeM) {
    // Seed both the filter and PID history from the first measurement after a reset.
    // This avoids a startup transient from stale/default state.
    if (!_filterInitialized) {
        _altitudeFilter.v = altitudeM;
        _pid.prevMeasurement = altitudeM;
        _pid.prevSetpoint = targetAltitudeM;
        _filterInitialized = true;
    }

    const float filteredAltitudeM = _altitudeFilter.update(altitudeM);
    const float correction = _pid.update(targetAltitudeM, filteredAltitudeM);

    _debug.filteredAltitudeM = filteredAltitudeM;
    _debug.errorM = _pid.error;
    _debug.pTerm = _pid.pTerm;
    _debug.iTerm = _pid.iTerm;
    _debug.dTerm = _pid.dTerm;
    _debug.correction = correction;

    return std::clamp(HOVER_THROTTLE + correction, 0.0f, 1.0f);
}

const AltitudeControllerDebug& AltitudeController::debug() const {
    return _debug;
}

}
