#pragma once

#include <Control/Pid.h>

namespace DronePet {

struct XyControlOutput {
    float roll = 0.0f;
    float pitch = 0.0f;

    float unclampedRollCommand = 0.0f;
    float unclampedPitchCommand = 0.0f;
    float rollITerm = 0.0f;
    float pitchITerm = 0.0f;
};

class XyVelocityController {
public:
    void begin();
    void reset();

    XyControlOutput update(
        float targetVelocityXMps,
        float targetVelocityYMps,
        float velocityXMps,
        float velocityYMps
    );

private:
    Espfc::Control::Pid _xPid;
    Espfc::Control::Pid _yPid;
};

}