#include "VelocityEstimator.h"

#include <cmath>

namespace {

constexpr float FLOW_NOISE_STD_MPS = 0.20f;
constexpr float PROCESS_ACCEL_STD_MPS2 = 4.0f;
constexpr float INNOVATION_GATE_SIGMA = 3.0f;

constexpr float MAX_DT_S = 0.05f;

constexpr float FLOW_VARIANCE = FLOW_NOISE_STD_MPS * FLOW_NOISE_STD_MPS;

}

void DronePet::VelocityEstimator::reset(uint32_t sensorTimeMs) {
    _worldX = {};
    _worldY = {};

    _worldX.variance = FLOW_VARIANCE;
    _worldY.variance = FLOW_VARIANCE;

    _state = {};

    _lastSensorTimeMs = sensorTimeMs;
    _initialized = true;
}

const DronePet::VelocityEstimate& DronePet::VelocityEstimator::update(
    uint32_t sensorTimeMs,
    float flowVelocityXMps,
    float flowVelocityYMps,
    float yawRad
) {
    if (!_initialized) {
        reset(sensorTimeMs);
        return _state;
    }

    const uint32_t dtMs = sensorTimeMs - _lastSensorTimeMs;
    _lastSensorTimeMs = sensorTimeMs;

    float dt = dtMs / 1000.0f;

    if (dt <= 0.0f) {
        return _state;
    }

    if (dt > MAX_DT_S) {
        dt = MAX_DT_S;
    }

    // DronePet:
    // +X = left
    // +Y = back
    //
    // ESP-FC body:
    // +X = forward
    // +Y = right
    const float flowForwardMps = -flowVelocityYMps;
    const float flowRightMps = -flowVelocityXMps;

    const float c = std::cos(yawRad);
    const float s = std::sin(yawRad);

    // Rotate optical-flow velocity into ESP-FC's world frame.
    const float flowWorldXMps =
        c * flowForwardMps - s * flowRightMps;

    const float flowWorldYMps =
        s * flowForwardMps + c * flowRightMps;

    updateAxis(
        _worldX,
        flowWorldXMps,
        dt,
        _state.innovationXMps,
        _state.innovationRatioX,
        _state.flowAcceptedX
    );

    updateAxis(
        _worldY,
        flowWorldYMps,
        dt,
        _state.innovationYMps,
        _state.innovationRatioY,
        _state.flowAcceptedY
    );

    // Convert estimated world velocity back into body coordinates.
    const float estimatedForwardMps =
        c * _worldX.velocityMps
        + s * _worldY.velocityMps;

    const float estimatedRightMps =
        -s * _worldX.velocityMps
        + c * _worldY.velocityMps;

    // Back into DronePet axes.
    _state.velocityXMps = -estimatedRightMps;
    _state.velocityYMps = -estimatedForwardMps;

    return _state;
}

void DronePet::VelocityEstimator::updateAxis(
    AxisState& axis,
    float measuredVelocityMps,
    float dt,
    float& innovationMps,
    float& innovationRatio,
    bool& measurementAccepted
) {
    // Constant-velocity prediction.
    // Unknown physical acceleration increases our uncertainty,
    // but measured IMU acceleration does not directly move the estimate.
    const float velocityProcessStd = PROCESS_ACCEL_STD_MPS2 * dt;

    axis.variance += velocityProcessStd * velocityProcessStd;

    // Compare optical flow against prediction.
    innovationMps =
        measuredVelocityMps - axis.velocityMps;

    const float innovationVariance =
        axis.variance + FLOW_VARIANCE;

    const float innovationStd =
        std::sqrt(innovationVariance);

    innovationRatio =
        innovationStd > 0.0f
            ? std::fabs(innovationMps) / innovationStd
            : 0.0f;

    measurementAccepted =
        innovationRatio <= INNOVATION_GATE_SIGMA;

    if (!measurementAccepted) {
        return;
    }

    // Kalman correction.
    const float gain =
        axis.variance / innovationVariance;

    axis.velocityMps +=
        gain * innovationMps;

    axis.variance *=
        1.0f - gain;
}

const DronePet::VelocityEstimate& DronePet::VelocityEstimator::state() const {
    return _state;
}
