#pragma once

struct FlightLoopIteration;
struct GyroCalibrationResult;
struct ImuHealthStatus;
struct ImuSample;
struct TofHealthStatus;
struct TofSample;

namespace Telemetry {
  void printImuInitializationFailed();
  void printImuInitialized();
  void printTofInitializationFailed();
  void printTofInitialized();
  void printGyroCalibrationPrompt();

  void printGyroCalibrationResult(
      const GyroCalibrationResult& result
  );

  void printFlightLoopStarted();

  void printFlightSample(
      const ImuSample& imuSample,
      const FlightLoopIteration& iteration,
      const ImuHealthStatus& imuHealth,
      const TofSample& tofSample,
      const TofHealthStatus& tofHealth
  );
}
