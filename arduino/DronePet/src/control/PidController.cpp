#include <Arduino.h>

#include "PidController.h"


PidController::PidController(
    PidGains gains,
    float outputLimit,
    float integralLimit
)
    : gains_(gains),
      outputLimit_(outputLimit),
      integralLimit_(integralLimit) {
}

void PidController::reset() {

  integral_ = 0.0f;

  previousError_ = 0.0f;

  hasPreviousError_ = false;
}

float PidController::update(
    float error,
    float dtSeconds
) {

  if (!isfinite(error)
      || !isfinite(dtSeconds)
      || dtSeconds <= 0.0f) {

    return 0.0f;
  }


  // Integral
  integral_ += error * dtSeconds;

  integral_ = constrain(
      integral_,
      -integralLimit_,
      integralLimit_
  );


  // Derivative
  float derivative = 0.0f;

  if (hasPreviousError_) {

    derivative =
        (error - previousError_)
        / dtSeconds;
  }


  previousError_ = error;
  hasPreviousError_ = true;


  // PID
  const float output =
      gains_.kp * error
      + gains_.ki * integral_
      + gains_.kd * derivative;


  return constrain(
      output,
      -outputLimit_,
      outputLimit_
  );
}
