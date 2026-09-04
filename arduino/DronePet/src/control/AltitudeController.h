#pragma once

struct TofSample;
struct TofHealthStatus;

struct AltitudeControlOutput {
  float targetHeightMm = 0.0f;
  float measuredHeightMm = 0.0f;
  float errorMm = 0.0f;

  float correctionPercent = 0.0f;
  float collectiveThrottlePercent = 0.0f;

  bool valid = false;
};

namespace AltitudeController {

  void begin();

  void reset();

  void setTargetHeightMm(float targetHeightMm);

  float targetHeightMm();

  AltitudeControlOutput update(
      const TofSample& sample,
      const TofHealthStatus& health
  );
}