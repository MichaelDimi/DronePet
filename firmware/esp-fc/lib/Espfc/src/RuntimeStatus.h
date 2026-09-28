#pragma once

namespace Espfc {

struct RuntimeStatus {
    bool fresh = false;
    bool ready = false;

    bool armed = false;
    bool angleMode = false;

    float rollDeg = 0.0f;
    float pitchDeg = 0.0f;
    float yawRad = 0.0f;

    float rollRateRadS = 0.0f;
    float pitchRateRadS = 0.0f;

    float accelWorldXMps2 = 0.0f;
    float accelWorldYMps2 = 0.0f;

    float rollRateSetpointRadS = 0.0f;
    float pitchRateSetpointRadS = 0.0f;
};

}