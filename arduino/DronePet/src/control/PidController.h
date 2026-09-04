#pragma once

struct PidGains {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
};

class PidController {

public:

  PidController(
      PidGains gains,
      float outputLimit,
      float integralLimit
  );

  void reset();

  float update(
      float error,
      float dtSeconds
  );


private:

  PidGains gains_;

  float outputLimit_;
  float integralLimit_;

  float integral_ = 0.0f;
  float previousError_ = 0.0f;

  bool hasPreviousError_ = false;
};