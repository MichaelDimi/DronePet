#include <math.h>

#include "ImuHealth.h"

namespace {
    // Values near the configured full-scale limits cannot be trusted because
    // the real motion may extend beyond what the sensor can represent.
    constexpr float ACCEL_AXIS_LIMIT_G = 3.9f;
    constexpr float GYRO_SATURATION_LIMIT_DPS = 490.0f;

    // These are intentionally broad initial safety limits, not assumptions
    // that the acceleration should always equal exactly 1 g.
    constexpr float MIN_ACCEL_MAGNITUDE_G = 0.25f;
    constexpr float MAX_ACCEL_MAGNITUDE_G = 3.5f;

    bool vectorIsFinite(const Vector3& value) {
        return isfinite(value.x)
            && isfinite(value.y)
            && isfinite(value.z);
    }

    bool allAxesWithin(
        const Vector3& value,
        float absoluteLimit
    ) {
        return fabsf(value.x) < absoluteLimit
            && fabsf(value.y) < absoluteLimit
            && fabsf(value.z) < absoluteLimit;
    }
}

ImuHealthStatus ImuHealth::evaluate(
    bool spiReadOk,
    const ImuSample& sample
) {
    ImuHealthStatus health;
    health.spiReadOk = spiReadOk;

    if (!spiReadOk) {
        return health;
    }

    health.accelerationValid =
      vectorIsFinite(sample.accelG)
      && allAxesWithin(sample.accelG, ACCEL_AXIS_LIMIT_G);

    // A non-finite gyro value is treated like saturation because either
    // condition makes the angular-rate measurement unusable.
    health.gyroSaturated =
        !vectorIsFinite(sample.gyroDps)
        || !allAxesWithin(
            sample.gyroDps,
            GYRO_SATURATION_LIMIT_DPS
        );

    health.accelMagnitudeG = sqrtf(
        sample.accelG.x * sample.accelG.x
        + sample.accelG.y * sample.accelG.y
        + sample.accelG.z * sample.accelG.z
    );

    health.accelMagnitudeValid =
        isfinite(health.accelMagnitudeG)
        && health.accelMagnitudeG >= MIN_ACCEL_MAGNITUDE_G
        && health.accelMagnitudeG <= MAX_ACCEL_MAGNITUDE_G;

    return health;
}