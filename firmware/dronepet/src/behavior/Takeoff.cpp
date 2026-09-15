#include "Takeoff.h"

#include "../sensors/Mtf01.h"

#include <Arduino.h>
#include <cmath>

namespace {

constexpr float TARGET_ALTITUDE_M = 0.35f;
constexpr float TARGET_TOLERANCE_M = 0.02f;

constexpr float TAKEOFF_THROTTLE = 0.33f;

constexpr uint32_t ARM_SETTLE_MS = 200;
constexpr uint32_t ALTITUDE_ACQUIRE_TIMEOUT_MS = 1200;

constexpr float MAX_SAFE_ALTITUDE_M = 0.50f;
constexpr float MAX_TILT_DEG = 25.0f;

bool _altitudeAcquired = false;


bool readAltitude(float& altitudeM) {
    if (!DronePet::Mtf01::fresh()) return false;

    const auto& sample = DronePet::Mtf01::latestSample();

    if (sample.distanceMm < 10) return false;

    altitudeM = sample.distanceMm / 1000.0f;
    return true;
}

}


void DronePet::Takeoff::begin(AltitudeController& altitudeController) {
    altitudeController.reset();
    _altitudeAcquired = false;
    _armedAtMs = 0;
}


DronePet::BehaviorResult DronePet::Takeoff::update(
    const Espfc::RuntimeStatus& fcStatus,
    AltitudeController& altitudeController,
    PilotCommand& command
) {
    command = {};
    command.angleMode = true;
    command.armed = true;

    if (std::fabs(fcStatus.rollDeg) > MAX_TILT_DEG ||
        std::fabs(fcStatus.pitchDeg) > MAX_TILT_DEG) {
        return BehaviorResult::SafeLanding;
    }

    if (!fcStatus.ready) {
        if (fcStatus.armed) return BehaviorResult::SafeLanding;

        command.armed = false;
        return BehaviorResult::Running;
    }

    if (!fcStatus.armed) return BehaviorResult::Running;

    const uint32_t nowMs = millis();

    if (_armedAtMs == 0) _armedAtMs = nowMs;

    if (nowMs - _armedAtMs < ARM_SETTLE_MS) return BehaviorResult::Running;

    float altitudeM = 0.0f;

    if (readAltitude(altitudeM)) {
        _altitudeAcquired = true;

        if (altitudeM > MAX_SAFE_ALTITUDE_M) return BehaviorResult::SafeLanding;

        command.throttle = altitudeController.update(TARGET_ALTITUDE_M, altitudeM);

        if (altitudeM >= TARGET_ALTITUDE_M - TARGET_TOLERANCE_M) {
            return BehaviorResult::Complete;
        }

        return BehaviorResult::Running;
    }

    if (_altitudeAcquired) return BehaviorResult::SafeLanding;

    // Only allowed before the first usable altitude reading.
    command.throttle = TAKEOFF_THROTTLE;

    if (nowMs - _armedAtMs > ALTITUDE_ACQUIRE_TIMEOUT_MS) {
        return BehaviorResult::SafeLanding;
    }

    return BehaviorResult::Running;
}