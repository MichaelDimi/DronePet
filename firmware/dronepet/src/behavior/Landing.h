#pragma once

#include <cstdint>

#include "BehaviorResult.h"
#include "../controller/AltitudeController.h"
#include "../flight/PilotCommand.h"

#include <RuntimeStatus.h>

namespace DronePet {

class Landing {
public:
    void begin(AltitudeController& altitudeController);

    BehaviorResult update(
        const Espfc::RuntimeStatus& fcStatus,
        AltitudeController& altitudeController,
        PilotCommand& command
    );

private:
    enum class Stage {
        Guided,
        FinalSettle
    };

    Stage _stage = Stage::Guided;

    bool _reachedLowRegion = false;

    float _lastThrottle = 0.30f;
    float _finalThrottle = 0.30f;

    uint32_t _lastUpdateMs = 0;
};

}