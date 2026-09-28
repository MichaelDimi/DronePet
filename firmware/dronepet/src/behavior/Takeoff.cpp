#include "Takeoff.h"

#include "../sensors/Mtf01.h"

#include <Arduino.h>
#include <cmath>

namespace {

constexpr float TAKEOFF_DELTA_M = 0.10f;
constexpr float TARGET_TOLERANCE_M = 0.02f;
constexpr uint32_t TAKEOFF_TIMEOUT_MS = 3000;

constexpr float MAX_TILT_DEG = 25.0f;

}

bool DronePet::Takeoff::begin(AltitudeController& altitudeController) {
    if (!Mtf01::fresh()) return false;

    const auto& sample = Mtf01::latestSample();
    if (sample.distanceMm < 10) return false;

    _targetAltitudeM = sample.distanceMm / 1000.0f + TAKEOFF_DELTA_M;
    _startMs = millis();

    altitudeController.reset();
    return true;
}

DronePet::BehaviorResult DronePet::Takeoff::update(
    const Espfc::RuntimeStatus& fcStatus,
    AltitudeController& altitudeController,
    PilotCommand& command
) {
    command = {};
    command.armed = true;
    command.angleMode = true;

    if (!fcStatus.ready || !fcStatus.armed) return BehaviorResult::SafeLanding;
    if (std::fabs(fcStatus.rollDeg) > MAX_TILT_DEG || std::fabs(fcStatus.pitchDeg) > MAX_TILT_DEG) return BehaviorResult::SafeLanding;

    if (millis() - _startMs >= TAKEOFF_TIMEOUT_MS) return BehaviorResult::SafeLanding;

    if (!Mtf01::fresh()) return BehaviorResult::SafeLanding;

    const auto& sample = Mtf01::latestSample();
    if (sample.distanceMm < 10) return BehaviorResult::SafeLanding;

    const float altitudeM = sample.distanceMm / 1000.0f;
    command.throttle = altitudeController.update(_targetAltitudeM, altitudeM);

    if (altitudeM >= _targetAltitudeM - TARGET_TOLERANCE_M) return BehaviorResult::Complete;

    return BehaviorResult::Running;
}