#pragma once

struct AttitudeControlOutput;

struct MotorMixOutput {
  float motor1Percent = 0.0f;
  float motor2Percent = 0.0f;
  float motor3Percent = 0.0f;
  float motor4Percent = 0.0f;

  bool valid = false;
};

namespace MotorMixer {

  MotorMixOutput mix(
      float collectiveThrottlePercent,
      const AttitudeControlOutput& attitude
  );

}