#pragma once

struct ImuSample;

struct AttitudeState {
  float rollDeg = 0.0f;
  float pitchDeg = 0.0f;

  float rollRateDps = 0.0f;
  float pitchRateDps = 0.0f;
  float yawRateDps = 0.0f;

  bool ready = false;
};

namespace AttitudeEstimator {
  // Reset the estimator and begin learning the level reference.
  // The drone should remain flat and still until state().ready becomes true.
  void begin();

  // Update the estimate from one calibrated IMU sample.
  void update(const ImuSample& sample, float dtSeconds);

  const AttitudeState& state();
}
