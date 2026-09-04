#include <Arduino.h>
#include <math.h>

#include "AttitudeEstimator.h"
#include "../imu/Imu.h"

namespace {
  constexpr size_t LEVEL_REFERENCE_SAMPLES = 100;

  // Time constant for blending gyro integration with the accelerometer's
  // long-term gravity reference. At 200 Hz this produces alpha ~= 0.99.
  constexpr float COMPLEMENTARY_TIME_CONSTANT_S = 0.5f;

  // Only use the accelerometer as a gravity reference when its total
  // magnitude is reasonably close to 1 g.
  constexpr float MIN_GRAVITY_REFERENCE_G = 0.75f;
  constexpr float MAX_GRAVITY_REFERENCE_G = 1.25f;

  AttitudeState currentState;

  size_t levelReferenceSampleCount = 0;
  float rollReferenceSumDeg = 0.0f;
  float pitchReferenceSumDeg = 0.0f;
  float rollReferenceDeg = 0.0f;
  float pitchReferenceDeg = 0.0f;

  float radiansToDegrees(float radians) {
    return radians * 180.0f / PI;
  }

  float accelMagnitudeG(const ImuSample& sample) {
    return sqrtf(
        sample.accelG.x * sample.accelG.x
        + sample.accelG.y * sample.accelG.y
        + sample.accelG.z * sample.accelG.z
    );
  }

  bool gravityReferenceUsable(const ImuSample& sample) {
    const float magnitude = accelMagnitudeG(sample);

    return isfinite(magnitude)
        && magnitude >= MIN_GRAVITY_REFERENCE_G
        && magnitude <= MAX_GRAVITY_REFERENCE_G;
  }

  // IMU mounting / body convention:
  //   +X = left
  //   -Y = forward
  //   +Z = up
  //
  // Therefore:
  //   positive roll  = rotation about forward (-Y)
  //                  = right side moves down
  //   positive pitch = rotation about left (+X)
  //                  = nose moves down
  //
  // Near level, those motions produce +X and +Y accelerometer tilt
  // respectively, so these equations give matching positive signs.
  float rollFromAccelDeg(const ImuSample& sample) {
    return radiansToDegrees(
        atan2f(
            sample.accelG.x,
            sqrtf(
                sample.accelG.y * sample.accelG.y
                + sample.accelG.z * sample.accelG.z
            )
        )
    );
  }

  float pitchFromAccelDeg(const ImuSample& sample) {
    return radiansToDegrees(
        atan2f(
            sample.accelG.y,
            sqrtf(
                sample.accelG.x * sample.accelG.x
                + sample.accelG.z * sample.accelG.z
            )
        )
    );
  }
}

void AttitudeEstimator::begin() {
  currentState = {};

  levelReferenceSampleCount = 0;
  rollReferenceSumDeg = 0.0f;
  pitchReferenceSumDeg = 0.0f;
  rollReferenceDeg = 0.0f;
  pitchReferenceDeg = 0.0f;
}

void AttitudeEstimator::update(
    const ImuSample& sample,
    float dtSeconds
) {
  // Convert native IMU gyro axes to our body rotations.
  // Forward is sensor -Y, left is sensor +X, and up is sensor +Z.
  currentState.rollRateDps = -sample.gyroDps.y;
  currentState.pitchRateDps = sample.gyroDps.x;
  currentState.yawRateDps = sample.gyroDps.z;

  const bool accelUsable = gravityReferenceUsable(sample);

  if (!currentState.ready) {
    if (!accelUsable) {
      return;
    }

    rollReferenceSumDeg += rollFromAccelDeg(sample);
    pitchReferenceSumDeg += pitchFromAccelDeg(sample);
    levelReferenceSampleCount++;

    if (levelReferenceSampleCount >= LEVEL_REFERENCE_SAMPLES) {
      const float sampleCount =
          static_cast<float>(levelReferenceSampleCount);

      rollReferenceDeg = rollReferenceSumDeg / sampleCount;
      pitchReferenceDeg = pitchReferenceSumDeg / sampleCount;

      currentState.rollDeg = 0.0f;
      currentState.pitchDeg = 0.0f;
      currentState.ready = true;
    }

    return;
  }

  if (!isfinite(dtSeconds) || dtSeconds <= 0.0f) {
    return;
  }

  // First propagate the estimate using the gyro. This gives us the fast,
  // responsive part of the attitude estimate.
  const float gyroRollDeg =
      currentState.rollDeg
      + currentState.rollRateDps * dtSeconds;

  const float gyroPitchDeg =
      currentState.pitchDeg
      + currentState.pitchRateDps * dtSeconds;

  if (!accelUsable) {
    currentState.rollDeg = gyroRollDeg;
    currentState.pitchDeg = gyroPitchDeg;
    return;
  }

  // Then gently pull the estimate back toward gravity so gyro bias cannot
  // cause unbounded roll/pitch drift.
  const float accelRollDeg =
      rollFromAccelDeg(sample) - rollReferenceDeg;

  const float accelPitchDeg =
      pitchFromAccelDeg(sample) - pitchReferenceDeg;

  const float alpha =
      COMPLEMENTARY_TIME_CONSTANT_S
      / (COMPLEMENTARY_TIME_CONSTANT_S + dtSeconds);

  currentState.rollDeg =
      alpha * gyroRollDeg
      + (1.0f - alpha) * accelRollDeg;

  currentState.pitchDeg =
      alpha * gyroPitchDeg
      + (1.0f - alpha) * accelPitchDeg;
}

const AttitudeState& AttitudeEstimator::state() {
  return currentState;
}
