#pragma once

#include <cstdint>

#include "BehaviorResult.h"
#include "../controller/AltitudeController.h"
#include "../flight/PilotCommand.h"

#include <RuntimeStatus.h>

namespace DronePet {

class Takeoff {
public:
    void begin(AltitudeController& altitudeController);

    BehaviorResult update(
        const Espfc::RuntimeStatus& fcStatus,
        AltitudeController& altitudeController,
        PilotCommand& command
    );

private:
    uint32_t _armedAtMs = 0;
};

}