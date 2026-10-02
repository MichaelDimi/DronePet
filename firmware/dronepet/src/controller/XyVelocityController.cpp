#include "XyVelocityController.h"

#include <cmath>

namespace {

constexpr float CONTROL_RATE_HZ = 100.0f;

constexpr float VELOCITY_KP = 0.22f; // Previous good value: 0.20
constexpr float VELOCITY_KI = 0.30f; // Previous good value: 0.3
constexpr float VELOCITY_KD = 0.002f; // Previous good value: 0.003

constexpr float I_LIMIT = 0.30f; // Previous good value: 0.20
constexpr float MAX_COMMAND = 0.30f; // Previous good value: 0.15

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

namespace DronePet {

void XyVelocityController::begin() {
    configurePid(_xPid);
    configurePid(_yPid);
}

void XyVelocityController::reset() {
    _xPid.resetIterm();
    _yPid.resetIterm();
    _xPid.outputSaturated = false;
    _yPid.outputSaturated = false;
}

XyControlOutput XyVelocityController::update(float targetVelocityXMps, float targetVelocityYMps,
                                             float velocityXMps, float velocityYMps) {
    const float xCorrection = _xPid.update(targetVelocityXMps, velocityXMps);
    const float yCorrection = _yPid.update(targetVelocityYMps, velocityYMps);

    // The PID returns a clamped correction. Reconstruct the raw correction so the
    // PID can stop accumulating I on the following update while its output is saturated.
    const float unclampedXCorrection = _xPid.pTerm + _xPid.iTerm + _xPid.dTerm + _xPid.fTerm;
    const float unclampedYCorrection = _yPid.pTerm + _yPid.iTerm + _yPid.dTerm + _yPid.fTerm;
    _xPid.outputSaturated = std::fabs(unclampedXCorrection) >= MAX_COMMAND;
    _yPid.outputSaturated = std::fabs(unclampedYCorrection) >= MAX_COMMAND;

    XyControlOutput output;

    // Convert DronePet body axes (+X left, +Y back) to the FC roll/pitch commands.
    output.roll = -xCorrection;
    output.pitch = -yCorrection;
    output.unclampedRollCommand = -unclampedXCorrection;
    output.unclampedPitchCommand = -unclampedYCorrection;
    output.rollITerm = -_xPid.iTerm;
    output.pitchITerm = -_yPid.iTerm;

    return output;
}

}
