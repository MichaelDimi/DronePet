#include "Landing.h"

#include "../sensors/Mtf01.h"

#include <Arduino.h>
#include <algorithm>
#include <cmath>

namespace {

constexpr float LANDING_TARGET_M = 0.08f;
constexpr float LOW_REGION_M = 0.10f;

constexpr float FINAL_THROTTLE_RAMP_PER_SEC = 0.12f;

constexpr float MAX_TILT_DEG = 25.0f;

}

void DronePet::Landing::begin(AltitudeController& altitudeController) {
    altitudeController.reset();

    _stage = Stage::Guided;
    _reachedLowRegion = false;

    _lastThrottle = 0.30f;
    _finalThrottle = 0.30f;

    _lastUpdateMs = millis();
}

DronePet::BehaviorResult DronePet::Landing::update(
    const Espfc::RuntimeStatus& fcStatus,
    AltitudeController& altitudeController,
    PilotCommand& command
) {
    command = {};
    command.armed = true;
    command.angleMode = true;

    if (!fcStatus.ready || !fcStatus.armed) return BehaviorResult::SafeLanding;
    if (std::fabs(fcStatus.rollDeg) > MAX_TILT_DEG || std::fabs(fcStatus.pitchDeg) > MAX_TILT_DEG) return BehaviorResult::SafeLanding;

    const uint32_t nowMs = millis();

    if (_stage == Stage::Guided) {
        if (Mtf01::fresh()) {
            const auto& sample = Mtf01::latestSample();

            if (sample.distanceMm >= 10) {
                const float altitudeM = sample.distanceMm / 1000.0f;

                if (altitudeM <= LOW_REGION_M) _reachedLowRegion = true;

                _lastThrottle = altitudeController.update(LANDING_TARGET_M, altitudeM);
                command.throttle = _lastThrottle;

                if (altitudeM <= LANDING_TARGET_M) {
                    _stage = Stage::FinalSettle;
                    _finalThrottle = _lastThrottle;
                    _lastUpdateMs = nowMs;
                }

                return BehaviorResult::Running;
            }
        }

        if (!_reachedLowRegion) return BehaviorResult::SafeLanding;

        _stage = Stage::FinalSettle;
        _finalThrottle = _lastThrottle;
        _lastUpdateMs = nowMs;
        command.throttle = _finalThrottle;

        return BehaviorResult::Running;
    }

    const float dt = (nowMs - _lastUpdateMs) / 1000.0f;
    _lastUpdateMs = nowMs;

    _finalThrottle = std::max(0.0f, _finalThrottle - FINAL_THROTTLE_RAMP_PER_SEC * dt);
    command.throttle = _finalThrottle;

    if (_finalThrottle <= 0.0f) {
        command.armed = false;
        return BehaviorResult::Complete;
    }

    return BehaviorResult::Running;
}