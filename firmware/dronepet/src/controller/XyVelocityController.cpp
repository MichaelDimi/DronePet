#include "XyVelocityController.h"

namespace {

constexpr float CONTROL_RATE_HZ = 100.0f;
constexpr float VELOCITY_FILTER_HZ = 10.0f;

constexpr float VELOCITY_KP = 0.15f;
constexpr float VELOCITY_KI = 0.0f;
constexpr float VELOCITY_KD = 0.0f;

// Maximum normalized roll/pitch stick correction.
// Keep this conservative for the first test.
constexpr float MAX_COMMAND = 0.06f;


void configurePid(Espfc::Control::Pid& pid) {
    pid.rate = CONTROL_RATE_HZ;

    pid.Kp = VELOCITY_KP;
    pid.Ki = VELOCITY_KI;
    pid.Kd = VELOCITY_KD;
    pid.Kf = 0.0f;

    pid.oLimitLow = -MAX_COMMAND;
    pid.oLimitHigh = MAX_COMMAND;

    pid.iLimitLow = -MAX_COMMAND;
    pid.iLimitHigh = MAX_COMMAND;

    pid.begin();
}

}


void DronePet::XyVelocityController::begin() {
    _xFilter.init(CONTROL_RATE_HZ, VELOCITY_FILTER_HZ);
    _yFilter.init(CONTROL_RATE_HZ, VELOCITY_FILTER_HZ);

    configurePid(_xPid);
    configurePid(_yPid);

    _filterInitialized = false;
}


void DronePet::XyVelocityController::reset() {
    _xPid.resetIterm();
    _yPid.resetIterm();

    _filterInitialized = false;
}


DronePet::XyControlOutput DronePet::XyVelocityController::update(
    float targetVelocityXMps,
    float targetVelocityYMps,
    float velocityXMps,
    float velocityYMps
) {
    if (!_filterInitialized) {
        _xFilter.v = velocityXMps;
        _yFilter.v = velocityYMps;
        _filterInitialized = true;
    }

    const float filteredX = _xFilter.update(velocityXMps);
    const float filteredY = _yFilter.update(velocityYMps);

    const float xCorrection = _xPid.update(targetVelocityXMps, filteredX);
    const float yCorrection = _yPid.update(targetVelocityYMps, filteredY);

    XyControlOutput output;

    // DronePet axes:
    // +X = left, so moving left requires rolling right.
    output.roll = -xCorrection;

    // -Y = back, so moving back requires pitching forward.
    output.pitch = -yCorrection;

    return output;
}