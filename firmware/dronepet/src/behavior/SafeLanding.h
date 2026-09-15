#pragma once

#include <cstdint>

#include "BehaviorResult.h"
#include "../flight/PilotCommand.h"

#include <RuntimeStatus.h>

namespace DronePet {

class SafeLanding {
public:
    void begin(float currentThrottle);

    BehaviorResult update(
        const Espfc::RuntimeStatus& fcStatus,
        PilotCommand& command
    );

private:
    float _throttle = 0.0f;
    uint32_t _lastUpdateMs = 0;
};

}