#pragma once

#include <Arduino.h>

namespace DronePet {

struct VelocityEstimate {
    float rawVelocityXMps = 0.0f;
    float rawVelocityYMps = 0.0f;

    // DronePet body-frame velocities returned to the XY controller.
    float velocityXMps = 0.0f;
    float velocityYMps = 0.0f;

    // Roll/pitch-rate compensated optical-flow measurement in DronePet body axes.
    float compensatedVelocityXMps = 0.0f;
    float compensatedVelocityYMps = 0.0f;

    // Kalman diagnostics below are in the fixed ESP-FC world frame.
    float innovationXMps = 0.0f;
    float innovationYMps = 0.0f;
    float innovationRatioX = 0.0f;
    float innovationRatioY = 0.0f;
    bool flowAcceptedX = true;
    bool flowAcceptedY = true;
};

class VelocityEstimator {
public:
    void reset(uint32_t sensorTimeMs);

    const VelocityEstimate& update(
        uint32_t sensorTimeMs,
        float flowVelocityXAt1mMps,
        float flowVelocityYAt1mMps,
        float rangeM,
        float rollRateRadS,
        float pitchRateRadS,
        float yawRad
    );

    const VelocityEstimate& state() const;

private:
    struct Velocity2D {
        float x = 0.0f;
        float y = 0.0f;
    };

    static Velocity2D scaleFlowToRange(
        Velocity2D flowVelocityAt1m,
        float rangeM
    );

    // Optical-flow compensation and frame conversions.
    static Velocity2D compensateFlowRotation(
        Velocity2D rawBodyVelocity,
        float rangeM,
        float rollRateRadS,
        float pitchRateRadS
    );
    static Velocity2D bodyToWorld(Velocity2D bodyVelocity, float yawRad);
    static Velocity2D worldToBody(Velocity2D worldVelocity, float yawRad);

    struct WorldAxisState {
        float velocityMps = 0.0f;
        float variance = 0.0f;
    };

    struct AxisUpdateResult {
        float innovationMps = 0.0f;
        float innovationRatio = 0.0f;
        bool measurementAccepted = true;
    };

    float updateDeltaTime(uint32_t sensorTimeMs);
    void updateWorldEstimate(Velocity2D measuredWorldVelocity, float dt);
    void updateBodyEstimate(float yawRad);

    static AxisUpdateResult updateAxis(
        WorldAxisState& axis,
        float measuredVelocityMps,
        float dt
    );

    WorldAxisState _worldX{};
    WorldAxisState _worldY{};
    VelocityEstimate _state{};

    uint32_t _lastSensorTimeMs = 0;
    bool _initialized = false;
};

}
