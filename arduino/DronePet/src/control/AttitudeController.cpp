#include <Arduino.h>

#include "AttitudeController.h"

#include "AttitudeEstimator.h"
#include "PidController.h"


namespace {

  // ------------------------------------------------------------
  // Outer angle controller
  // ------------------------------------------------------------

  // Converts angle error into desired rotation rate.
  //
  // Example:
  //
  // roll = +10 deg
  // target = 0 deg
  //
  // error = -10 deg
  //
  // desired roll rate = -40 deg/s
  //
  // So the drone tries to rotate back toward level.

  constexpr float ANGLE_KP = 4.0f;

  constexpr float MAX_DESIRED_RATE_DPS = 120.0f;


  // ------------------------------------------------------------
  // Inner rate PID
  // ------------------------------------------------------------

  // VERY preliminary values.
  //
  // We are NOT flying with these yet.
  //
  // For the first test we use proportional control only so
  // we can verify correction signs cleanly.

  constexpr PidGains ROLL_RATE_GAINS{
      .kp = 0.05f,
      .ki = 0.0f,
      .kd = 0.0f
  };

  constexpr PidGains PITCH_RATE_GAINS{
      .kp = 0.05f,
      .ki = 0.0f,
      .kd = 0.0f
  };

  constexpr PidGains YAW_RATE_GAINS{
      .kp = 0.05f,
      .ki = 0.0f,
      .kd = 0.0f
  };


  // For now, prevent the attitude controller from requesting
  // more than +/- 20 percentage points of throttle correction.

  constexpr float MAX_CORRECTION_PERCENT = 20.0f;


  // Integral isn't active yet, but the PID object requires
  // an anti-windup limit.
  constexpr float INTEGRAL_LIMIT = 100.0f;


  PidController rollRatePid(
      ROLL_RATE_GAINS,
      MAX_CORRECTION_PERCENT,
      INTEGRAL_LIMIT
  );

  PidController pitchRatePid(
      PITCH_RATE_GAINS,
      MAX_CORRECTION_PERCENT,
      INTEGRAL_LIMIT
  );

  PidController yawRatePid(
      YAW_RATE_GAINS,
      MAX_CORRECTION_PERCENT,
      INTEGRAL_LIMIT
  );


  float clampDesiredRate(float rateDps) {

    return constrain(
        rateDps,
        -MAX_DESIRED_RATE_DPS,
        MAX_DESIRED_RATE_DPS
    );
  }

}


void AttitudeController::begin() {

  rollRatePid.reset();
  pitchRatePid.reset();
  yawRatePid.reset();
}


AttitudeControlOutput AttitudeController::update(
    const AttitudeState& attitude,
    float dtSeconds
) {

  AttitudeControlOutput output{};


  if (!attitude.ready) {
    return output;
  }


  // ------------------------------------------------------------
  // OUTER LOOP: angle -> desired angular rate
  // ------------------------------------------------------------

  constexpr float TARGET_ROLL_DEG = 0.0f;
  constexpr float TARGET_PITCH_DEG = 0.0f;

  const float rollAngleError =
      TARGET_ROLL_DEG
      - attitude.rollDeg;

  const float pitchAngleError =
      TARGET_PITCH_DEG
      - attitude.pitchDeg;


  output.desiredRollRateDps =
      clampDesiredRate(
          ANGLE_KP * rollAngleError
      );

  output.desiredPitchRateDps =
      clampDesiredRate(
          ANGLE_KP * pitchAngleError
      );


  // For now, we don't care about absolute yaw heading.
  //
  // We simply want the drone to stop rotating.

  output.desiredYawRateDps = 0.0f;


  // ------------------------------------------------------------
  // INNER LOOP: desired rate -> torque correction
  // ------------------------------------------------------------

  const float rollRateError =
      output.desiredRollRateDps
      - attitude.rollRateDps;

  const float pitchRateError =
      output.desiredPitchRateDps
      - attitude.pitchRateDps;

  const float yawRateError =
      output.desiredYawRateDps
      - attitude.yawRateDps;


  output.rollCorrectionPercent =
      rollRatePid.update(
          rollRateError,
          dtSeconds
      );

  output.pitchCorrectionPercent =
      pitchRatePid.update(
          pitchRateError,
          dtSeconds
      );

  output.yawCorrectionPercent =
      yawRatePid.update(
          yawRateError,
          dtSeconds
      );


  output.valid = true;

  return output;
}