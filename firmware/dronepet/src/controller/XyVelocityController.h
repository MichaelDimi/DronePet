#pragma once

#include <Control/Pid.h>
#include <Utils/Filter.h>

namespace DronePet {

struct XyControlOutput {
    float roll = 0.0f;
    float pitch = 0.0f;
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

    Espfc::Utils::FilterStatePt1 _xFilter;
    Espfc::Utils::FilterStatePt1 _yFilter;

    bool _filterInitialized = false;
};

}