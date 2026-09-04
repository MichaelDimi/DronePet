#pragma once

struct AttitudeState;


struct AttitudeControlOutput {

  // Desired body rotation rates produced by the
  // outer angle controller.
  float desiredRollRateDps = 0.0f;
  float desiredPitchRateDps = 0.0f;
  float desiredYawRateDps = 0.0f;


  // Corrections that will eventually go to the mixer.
  //
  // Units are percentage points of motor throttle.
  float rollCorrectionPercent = 0.0f;
  float pitchCorrectionPercent = 0.0f;
  float yawCorrectionPercent = 0.0f;


  bool valid = false;
};


namespace AttitudeController {

  void begin();

  AttitudeControlOutput update(
      const AttitudeState& attitude,
      float dtSeconds
  );

}