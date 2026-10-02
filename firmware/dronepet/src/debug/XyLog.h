#pragma once

#include <Arduino.h>

namespace DronePet::XyLog {

void beginRun();

void add(
    uint32_t elapsedMs,
    uint32_t sensorTimeMs,

    float rawVelocityXMps,
    float rawVelocityYMps,

    float flowVelocityXMps,
    float flowVelocityYMps,

    float estimatedVelocityXMps,
    float estimatedVelocityYMps,

    float innovationXMps,
    float innovationYMps,

    float innovationRatioX,
    float innovationRatioY,

    float accelWorldXMps2,
    float accelWorldYMps2,

    float rollCommand,
    float pitchCommand,

    float unclampedRollCommand,
    float unclampedPitchCommand,

    float rollITerm,
    float pitchITerm,

    float actualRollDeg,
    float actualPitchDeg,

    float rollRateRadS,
    float pitchRateRadS,

    float rollRateSetpointRadS,
    float pitchRateSetpointRadS,

    bool flowAcceptedX,
    bool flowAcceptedY,

    uint8_t flowQuality,
    uint8_t flowStatus
);

bool save();
void printSaved(Stream& out);

}