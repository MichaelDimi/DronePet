#include "VelocityEstimator.h"

#include <cmath>

namespace {

constexpr float FLOW_GYRO_COMP_GAIN = 1.0f;
constexpr float FLOW_NOISE_STD_MPS = 0.20f;
constexpr float PROCESS_ACCEL_STD_MPS2 = 4.0f;
constexpr float INNOVATION_GATE_SIGMA = 3.0f;
constexpr float MAX_DT_S = 0.05f;

constexpr float FLOW_VARIANCE = FLOW_NOISE_STD_MPS * FLOW_NOISE_STD_MPS;

}

namespace DronePet {

VelocityEstimator::Velocity2D VelocityEstimator::scaleFlowToRange(
    Velocity2D flowVelocityAt1m,
    float rangeM
) {
    return {
        flowVelocityAt1m.x * rangeM,
        flowVelocityAt1m.y * rangeM
    };
}

VelocityEstimator::Velocity2D VelocityEstimator::compensateFlowRotation(
    Velocity2D rawBodyVelocity,
    float rangeM,
    float rollRateRadS,
    float pitchRateRadS
) {
    return {
        rawBodyVelocity.x - FLOW_GYRO_COMP_GAIN * rangeM * rollRateRadS,
        rawBodyVelocity.y - FLOW_GYRO_COMP_GAIN * rangeM * pitchRateRadS
    };
}

VelocityEstimator::Velocity2D VelocityEstimator::bodyToWorld(
    Velocity2D bodyVelocity,
    float yawRad
) {
    // DronePet body: +X left, +Y back.
    // ESP-FC body:   +X forward, +Y right.
    const float forwardMps = -bodyVelocity.y;
    const float rightMps = -bodyVelocity.x;

    const float c = std::cos(yawRad);
    const float s = std::sin(yawRad);

    return {
        c * forwardMps - s * rightMps,
        s * forwardMps + c * rightMps
    };
}

VelocityEstimator::Velocity2D VelocityEstimator::worldToBody(
    Velocity2D worldVelocity,
    float yawRad
) {
    const float c = std::cos(yawRad);
    const float s = std::sin(yawRad);

    const float forwardMps = c * worldVelocity.x + s * worldVelocity.y;
    const float rightMps = -s * worldVelocity.x + c * worldVelocity.y;

    return {-rightMps, -forwardMps};
}

void VelocityEstimator::reset(uint32_t sensorTimeMs) {
    _worldX = {};
    _worldY = {};
    _worldX.variance = FLOW_VARIANCE;
    _worldY.variance = FLOW_VARIANCE;

    _state = {};
    _lastSensorTimeMs = sensorTimeMs;
    _initialized = true;
}

const VelocityEstimate& VelocityEstimator::update(
    uint32_t sensorTimeMs,
    float flowVelocityXAt1mMps,
    float flowVelocityYAt1mMps,
    float rangeM,
    float rollRateRadS,
    float pitchRateRadS,
    float yawRad
) {
    if (!_initialized) reset(sensorTimeMs);

    const Velocity2D rawBodyVelocity = scaleFlowToRange(
        {flowVelocityXAt1mMps, flowVelocityYAt1mMps},
        rangeM
    );

    _state.rawVelocityXMps = rawBodyVelocity.x;
    _state.rawVelocityYMps = rawBodyVelocity.y;

    const Velocity2D compensatedBody = compensateFlowRotation(
        rawBodyVelocity,
        rangeM,
        rollRateRadS,
        pitchRateRadS
    );

    _state.compensatedVelocityXMps = compensatedBody.x;
    _state.compensatedVelocityYMps = compensatedBody.y;

    const float dt = updateDeltaTime(sensorTimeMs);
    if (dt <= 0.0f) return _state;

    const Velocity2D measuredWorld =
        bodyToWorld(compensatedBody, yawRad);

    updateWorldEstimate(measuredWorld, dt);
    updateBodyEstimate(yawRad);

    return _state;
}

float VelocityEstimator::updateDeltaTime(uint32_t sensorTimeMs) {
    const uint32_t dtMs = sensorTimeMs - _lastSensorTimeMs;
    _lastSensorTimeMs = sensorTimeMs;

    float dt = dtMs / 1000.0f;
    if (dt <= 0.0f) return 0.0f;
    if (dt > MAX_DT_S) dt = MAX_DT_S;
    return dt;
}

void VelocityEstimator::updateWorldEstimate(Velocity2D measuredWorldVelocity, float dt) {
    const AxisUpdateResult xResult = updateAxis(_worldX, measuredWorldVelocity.x, dt);
    const AxisUpdateResult yResult = updateAxis(_worldY, measuredWorldVelocity.y, dt);

    _state.innovationXMps = xResult.innovationMps;
    _state.innovationYMps = yResult.innovationMps;
    _state.innovationRatioX = xResult.innovationRatio;
    _state.innovationRatioY = yResult.innovationRatio;
    _state.flowAcceptedX = xResult.measurementAccepted;
    _state.flowAcceptedY = yResult.measurementAccepted;
}

void VelocityEstimator::updateBodyEstimate(float yawRad) {
    const Velocity2D estimatedBody = worldToBody(
        {_worldX.velocityMps, _worldY.velocityMps}, yawRad
    );

    _state.velocityXMps = estimatedBody.x;
    _state.velocityYMps = estimatedBody.y;
}

VelocityEstimator::AxisUpdateResult VelocityEstimator::updateAxis(
    WorldAxisState& axis,
    float measuredVelocityMps,
    float dt
) {
    AxisUpdateResult result{};

    // Constant-velocity prediction: unknown physical acceleration increases
    // uncertainty, but measured IMU acceleration does not move the estimate.
    const float velocityProcessStd = PROCESS_ACCEL_STD_MPS2 * dt;
    axis.variance += velocityProcessStd * velocityProcessStd;

    // Compare the optical-flow measurement against the prediction and reject
    // measurements that fall outside the innovation gate.
    result.innovationMps = measuredVelocityMps - axis.velocityMps;
    const float innovationVariance = axis.variance + FLOW_VARIANCE;
    const float innovationStd = std::sqrt(innovationVariance);

    result.innovationRatio = innovationStd > 0.0f
        ? std::fabs(result.innovationMps) / innovationStd
        : 0.0f;
    result.measurementAccepted = result.innovationRatio <= INNOVATION_GATE_SIGMA;
    if (!result.measurementAccepted) return result;

    // Kalman correction.
    const float gain = axis.variance / innovationVariance;
    axis.velocityMps += gain * result.innovationMps;
    axis.variance *= 1.0f - gain;

    return result;
}

const VelocityEstimate& VelocityEstimator::state() const {
    return _state;
}

}
