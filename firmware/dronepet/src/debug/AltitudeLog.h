#pragma once

#include <Arduino.h>

namespace DronePet::AltitudeLog {

void beginRun();
void add(
    uint32_t elapsedMs,
    float targetAltitudeM,
    float altitudeM,
    float filteredAltitudeM,
    float errorM,
    float pTerm,
    float iTerm,
    float dTerm,
    float correction,
    float throttle,
    uint8_t strength,
    uint8_t precision,
    uint8_t tofStatus
);

bool save();
void printSaved(Stream& out);

}