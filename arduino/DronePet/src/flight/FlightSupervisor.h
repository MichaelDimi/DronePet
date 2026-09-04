#pragma once

struct AttitudeState;
struct ImuHealthStatus;
struct TofSample;
struct TofHealthStatus;


enum class FlightState {
  WaitingForReady,
  ArmWarning,
  Flying,
  Landing,
  Complete,
  Fault
};

namespace FlightSupervisor {

  void begin();

  void update(
      const AttitudeState& attitude,
      const ImuHealthStatus& imuHealth,
      const TofSample& tofSample,
      const TofHealthStatus& tofHealth,
      float dtSeconds
  );

  FlightState state();
}