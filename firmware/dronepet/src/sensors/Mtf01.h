
#pragma once

#include <Arduino.h>

namespace DronePet {

struct MtfSample {
    uint32_t sensorTimeMs = 0;

    uint32_t distanceMm = 0;

    float velocityXMps = 0.0f;
    float velocityYMps = 0.0f;

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
