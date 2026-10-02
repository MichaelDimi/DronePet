
#pragma once

#include <Arduino.h>

namespace DronePet {

struct MtfSample {
    uint32_t sensorTimeMs = 0;

    float rangeM = 0.0f;

    // MTF optical-flow velocity normalized to a 1 m measurement distance,
    // converted to DronePet body axes.
    float flowVelocityXAt1mMps = 0.0f;
    float flowVelocityYAt1mMps = 0.0f;

    uint8_t strength = 0;
    uint8_t precision = 0;
    uint8_t tofStatus = 0;

    uint8_t flowQuality = 0;
    uint8_t flowStatus = 0;

    uint32_t receivedAtUs = 0;
};


namespace Mtf01 {

bool begin();
bool update();

bool hasSample();
bool fresh();

const MtfSample& latestSample();

}

}
