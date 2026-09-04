#include <Arduino.h>

#include "TofHealth.h"

namespace {
    // Below 40 mm (4 cm / about 1.6 in), the sensor may detect a
    // target, but the reported distance is not guaranteed accurate.
    constexpr uint16_t MIN_RELIABLE_DISTANCE_MM = 40;

    // Measurements are expected every 50 ms.
    // Allow three missed periods before declaring the sample stale.
    constexpr uint32_t SAMPLE_STALE_AFTER_US = 150'000;
}

TofHealthStatus TofHealth::evaluate(
    bool initialized,
    bool hasSample,
    bool communicationError,
    const TofSample& sample
) {
    TofHealthStatus health;

    health.initialized = initialized;
    health.sampleReceived = hasSample;
    health.communicationError = communicationError;

    if (!hasSample) {
        return health;
    }

    const uint32_t sampleAgeUs = micros() - sample.timestampUs;
    health.sampleAgeMs = sampleAgeUs / 1000;
    health.sampleFresh = sampleAgeUs <= SAMPLE_STALE_AFTER_US;
    
    health.rangeValid = sample.rangeValid;

    health.distanceAboveMinimum = sample.distanceMm >= MIN_RELIABLE_DISTANCE_MM;

    return health;
}
