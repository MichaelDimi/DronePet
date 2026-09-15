#include "Hover.h"

#include "../sensors/Mtf01.h"

#include <cmath>

namespace {

constexpr float MAX_TILT_DEG = 25.0f;

}


void DronePet::Hover::begin(float currentAltitudeM, AltitudeController& altitudeController) {
    _targetAltitudeM = currentAltitudeM;
    altitudeController.reset();
}


DronePet::BehaviorResult DronePet::Hover::update(
    const Espfc::RuntimeStatus& fcStatus,
    AltitudeController& altitudeController,
    PilotCommand& command
) {
    command = {};
    command.armed = true;
    command.angleMode = true;

    if (!fcStatus.ready ||
        !fcStatus.armed ||
        std::fabs(fcStatus.rollDeg) > MAX_TILT_DEG ||
        std::fabs(fcStatus.pitchDeg) > MAX_TILT_DEG) {
        return BehaviorResult::SafeLanding;
    }

    if (!Mtf01::fresh()) return BehaviorResult::SafeLanding;

    const auto& sample = Mtf01::latestSample();

    if (sample.distanceMm < 10) return BehaviorResult::SafeLanding;

    const float altitudeM = sample.distanceMm / 1000.0f;

    command.throttle = altitudeController.update(_targetAltitudeM, altitudeM);

    return BehaviorResult::Running;
}