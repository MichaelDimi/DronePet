#include "AltitudeController.h"

#include <algorithm>


namespace {

constexpr float CONTROL_RATE_HZ = 100.0f;


// Approximate hover throttle from our recent tests.
constexpr float HOVER_THROTTLE = 0.35f;


constexpr float ALTITUDE_KP = 0.20f;
constexpr float ALTITUDE_KI = 0.0f;
constexpr float ALTITUDE_KD = 0.0f;

// Don't let altitude control make huge corrections
// while we're tuning it.
constexpr float MAX_CORRECTION = 0.08f;

}


void DronePet::AltitudeController::begin() {

    constexpr float ALTITUDE_FILTER_HZ = 10.0f;

    _altitudeFilter.init(CONTROL_RATE_HZ, ALTITUDE_FILTER_HZ);

    _filterInitialized = false;

    _pid.rate =
        CONTROL_RATE_HZ;

    _pid.Kp =
        ALTITUDE_KP;

    _pid.Ki =
        ALTITUDE_KI;

    _pid.Kd =
        ALTITUDE_KD;

    _pid.Kf = 0.0f;


    _pid.oLimitLow =
        -MAX_CORRECTION;

    _pid.oLimitHigh =
        MAX_CORRECTION;


    _pid.iLimitLow =
        -MAX_CORRECTION;

    _pid.iLimitHigh =
        MAX_CORRECTION;


    _pid.begin();
}


void DronePet::AltitudeController::reset() {
    _pid.resetIterm();
    _filterInitialized = false;
}


float DronePet::AltitudeController::update(
    float targetAltitudeM,
    float altitudeM
) {

    if (!_filterInitialized) {
        _altitudeFilter.v = altitudeM;

        _pid.prevMeasurement = altitudeM;
        _pid.prevSetpoint = targetAltitudeM;

        _filterInitialized = true;
    }

    const float filteredAltitudeM = _altitudeFilter.update(altitudeM);

    const float correction =
        _pid.update(
            targetAltitudeM,
            filteredAltitudeM
        );

    return std::clamp(
        HOVER_THROTTLE + correction,
        0.0f,
        1.0f
    );
}