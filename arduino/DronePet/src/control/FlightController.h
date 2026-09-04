#pragma once

struct TofSample;
struct AttitudeState;
struct ImuHealthStatus;
struct TofHealthStatus;

namespace FlightController {

  void begin();

  void arm();

  void disarm();

  bool armed();

  void setTargetAltitudeMm(float targetHeightMm);

  void update(
      const AttitudeState& attitude,
      const ImuHealthStatus& imuHealth,
      const TofSample& tofSample,
      const TofHealthStatus& tofHealth,
      float dtSeconds
  );
}