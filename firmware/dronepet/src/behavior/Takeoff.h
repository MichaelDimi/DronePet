#pragma once

#include "BehaviorResult.h"
#include "../controller/AltitudeController.h"
#include "../flight/PilotCommand.h"
#include <RuntimeStatus.h>
#include <cstdint>

namespace DronePet {

class Takeoff {
public:
    bool begin(AltitudeController& altitudeController);
    BehaviorResult update(const Espfc::RuntimeStatus& fcStatus, AltitudeController& altitudeController, PilotCommand& command);

    float targetAltitudeM() const { return _targetAltitudeM; }

private:
    float _targetAltitudeM = 0.0f;
    uint32_t _startMs = 0;
};
}