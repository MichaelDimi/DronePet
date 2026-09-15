#pragma once

#include "BehaviorResult.h"
#include "../controller/AltitudeController.h"
#include "../flight/PilotCommand.h"

#include <RuntimeStatus.h>

namespace DronePet {

class Hover {
public:
    void begin(float currentAltitudeM, AltitudeController& altitudeController);

    BehaviorResult update(
        const Espfc::RuntimeStatus& fcStatus,
        AltitudeController& altitudeController,
        PilotCommand& command
    );

private:
    float _targetAltitudeM = 0.0f;
};

}