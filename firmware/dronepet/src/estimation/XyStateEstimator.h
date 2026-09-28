#pragma once

#include <Arduino.h>

namespace DronePet {

struct XyStateEstimate {
    float positionXM = 0.0f;
    float positionYM = 0.0f;

    float velocityXMps = 0.0f;
    float velocityYMps = 0.0f;
};

class XyStateEstimator {
public:
    void reset(uint32_t sensorTimeMs);

    const XyStateEstimate& update(
        uint32_t sensorTimeMs,
        float rawVelocityXMps,
        float rawVelocityYMps,
        float altitudeM,
        float rollRateRadS,
        float pitchRateRadS
    );

    const XyStateEstimate& state() const;

private:
    XyStateEstimate _state{};
    uint32_t _lastSensorTimeMs = 0;
    bool _initialized = false;
};

}