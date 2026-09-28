#include <cmath>

#include "XyVelocityController.h"

namespace {

constexpr float CONTROL_RATE_HZ = 100.0f;

constexpr float VELOCITY_KP = 0.22f;
constexpr float VELOCITY_KI = 0.15f;
constexpr float VELOCITY_KD = 0.0f;

constexpr float I_LIMIT = 0.2f;
constexpr float MAX_COMMAND = 0.15f;

void configurePid(Espfc::Control::Pid& pid) {
    pid.rate = CONTROL_RATE_HZ;
    pid.Kp = VELOCITY_KP;
    pid.Ki = VELOCITY_KI;
    pid.Kd = VELOCITY_KD;
    pid.Kf = 0.0f;

    pid.oLimitLow = -MAX_COMMAND;
    pid.oLimitHigh = MAX_COMMAND;

    pid.iLimitLow = -I_LIMIT;
    pid.iLimitHigh = I_LIMIT;

    pid.begin();
}

}

void DronePet::XyVelocityController::begin() {
    configurePid(_xPid);
    configurePid(_yPid);
}

void DronePet::XyVelocityController::reset() {
    _xPid.resetIterm();
    _yPid.resetIterm();

    _xPid.outputSaturated = false;
    _yPid.outputSaturated = false;
}

DronePet::XyControlOutput DronePet::XyVelocityController::update(
    float targetVelocityXMps,
    float targetVelocityYMps,
    float velocityXMps,
    float velocityYMps
) {
    const float xCorrection = _xPid.update(targetVelocityXMps, velocityXMps);
    const float yCorrection = _yPid.update(targetVelocityYMps, velocityYMps);

    const float unclampedXCorrection = _xPid.pTerm + _xPid.iTerm + _xPid.dTerm + _xPid.fTerm;
    const float unclampedYCorrection = _yPid.pTerm + _yPid.iTerm + _yPid.dTerm + _yPid.fTerm;

    _xPid.outputSaturated = std::fabs(unclampedXCorrection) >= MAX_COMMAND;
    _yPid.outputSaturated = std::fabs(unclampedYCorrection) >= MAX_COMMAND;

    XyControlOutput output;

    // DronePet +X = left.
    output.roll = -xCorrection;

    // DronePet +Y = back.
    output.pitch = -yCorrection;

    output.unclampedRollCommand = -unclampedXCorrection;
    output.unclampedPitchCommand = -unclampedYCorrection;
    output.rollITerm = -_xPid.iTerm;
    output.pitchITerm = -_yPid.iTerm;

    return output;
}