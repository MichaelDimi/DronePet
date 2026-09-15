#pragma once

namespace Espfc {

struct RuntimeStatus {
    bool fresh = false;
    bool ready = false;

    bool armed = false;
    bool angleMode = false;

    float rollDeg = 0.0f;
    float pitchDeg = 0.0f;
};

}