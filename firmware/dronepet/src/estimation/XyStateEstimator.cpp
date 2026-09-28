#include "XyStateEstimator.h"

namespace {

constexpr float FLOW_GYRO_COMP_GAIN = 1.0f;
constexpr float MAX_DT_S = 0.05f;

}

void DronePet::XyStateEstimator::reset(uint32_t sensorTimeMs) {
    _state = {};
    _lastSensorTimeMs = sensorTimeMs;
    _initialized = true;
}

const DronePet::XyStateEstimate& DronePet::XyStateEstimator::update(
    uint32_t sensorTimeMs,
    float rawVelocityXMps,
    float rawVelocityYMps,
    float altitudeM,
    float rollRateRadS,
    float pitchRateRadS
) {
    if (!_initialized) {
        reset(sensorTimeMs);
        return _state;
    }

    const uint32_t dtMs =
        sensorTimeMs - _lastSensorTimeMs;

    _lastSensorTimeMs = sensorTimeMs;

    float dt = dtMs / 1000.0f;

    if (dt > MAX_DT_S) {
        dt = MAX_DT_S;
    }

    // Remove apparent optical-flow velocity caused by
    // rotation of the downward-facing sensor.
    _state.velocityXMps =
        rawVelocityXMps
        - FLOW_GYRO_COMP_GAIN
        * altitudeM
        * rollRateRadS;

    _state.velocityYMps =
        rawVelocityYMps
        - FLOW_GYRO_COMP_GAIN
        * altitudeM
        * pitchRateRadS;

    _state.positionXM +=
        _state.velocityXMps * dt;

    _state.positionYM +=
        _state.velocityYMps * dt;

    return _state;
}

const DronePet::XyStateEstimate&
DronePet::XyStateEstimator::state() const {
    return _state;
}