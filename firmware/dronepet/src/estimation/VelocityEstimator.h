#pragma once

#include <Arduino.h>

namespace DronePet {

struct VelocityEstimate {
    float velocityXMps = 0.0f;
    float velocityYMps = 0.0f;

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
        float flowVelocityXMps,
        float flowVelocityYMps,
        float yawRad
    );

    const VelocityEstimate& state() const;

private:
    struct AxisState {
        float velocityMps = 0.0f;
        float variance = 0.0f;
    };

    void updateAxis(
        AxisState& axis,
        float measuredVelocityMps,
        float dt,
        float& innovationMps,
        float& innovationRatio,
        bool& measurementAccepted
    );

    AxisState _worldX{};
    AxisState _worldY{};
    VelocityEstimate _state{};

    uint32_t _lastSensorTimeMs = 0;
    bool _initialized = false;
};

}