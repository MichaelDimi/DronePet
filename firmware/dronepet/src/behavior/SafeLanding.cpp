#include "SafeLanding.h"

#include <Arduino.h>
#include <algorithm>

namespace {

constexpr float THROTTLE_RAMP_PER_SEC = 0.05f;

}


void DronePet::SafeLanding::begin(float currentThrottle) {
    _throttle = std::clamp(currentThrottle, 0.0f, 1.0f);
    _lastUpdateMs = millis();
}


DronePet::BehaviorResult DronePet::SafeLanding::update(
    const Espfc::RuntimeStatus& fcStatus,
    PilotCommand& command
) {
    command = {};
    command.angleMode = true;

    if (!fcStatus.armed) {
        command.armed = false;
        return BehaviorResult::Complete;
    }

    command.armed = true;

    const uint32_t nowMs = millis();
    const float dt = (nowMs - _lastUpdateMs) / 1000.0f;

    _lastUpdateMs = nowMs;

    _throttle = std::max(0.0f, _throttle - THROTTLE_RAMP_PER_SEC * dt);
    command.throttle = _throttle;

    if (_throttle <= 0.0f) {
        command.armed = false;
        return BehaviorResult::Complete;
    }

    return BehaviorResult::Running;
}