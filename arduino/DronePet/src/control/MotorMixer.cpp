#include <Arduino.h>

#include "MotorMixer.h"
#include "AttitudeController.h"


MotorMixOutput MotorMixer::mix(
    float collectiveThrottlePercent,
    const AttitudeControlOutput& attitude
) {

  MotorMixOutput output{};

  if (!attitude.valid) {
    return output;
  }


  const float roll =
      attitude.rollCorrectionPercent;

  const float pitch =
      attitude.pitchCorrectionPercent;

  const float yaw =
      attitude.yawCorrectionPercent;


  // Motor layout:
  //
  //             FRONT
  //
  //       M2             M4
  //    Front Left     Front Right
  //
  //       M1             M3
  //     Back Left      Back Right
  //
  //
  // +roll  = right side down
  // +pitch = nose down
  // +yaw   = CCW viewed from above


  output.motor1Percent =
      collectiveThrottlePercent
      + roll
      + pitch
      - yaw;

  output.motor2Percent =
      collectiveThrottlePercent
      + roll
      - pitch
      + yaw;

  output.motor3Percent =
      collectiveThrottlePercent
      - roll
      + pitch
      + yaw;

  output.motor4Percent =
      collectiveThrottlePercent
      - roll
      - pitch
      - yaw;


  output.motor1Percent =
      constrain(output.motor1Percent, 0.0f, 100.0f);

  output.motor2Percent =
      constrain(output.motor2Percent, 0.0f, 100.0f);

  output.motor3Percent =
      constrain(output.motor3Percent, 0.0f, 100.0f);

  output.motor4Percent =
      constrain(output.motor4Percent, 0.0f, 100.0f);


  output.valid = true;

  return output;
}