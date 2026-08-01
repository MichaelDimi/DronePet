#pragma once

#include "../flight/FlightLoop.h"
#include "../imu/Imu.h"
#include "../imu/ImuStartup.h"
#include "../imu/ImuHealth.h"

namespace Telemetry {
    void printImuInitializationFailed();
    void printImuInitialized();
    void printGyroCalibrationPrompt();

    void printGyroCalibrationResult(
        const GyroCalibrationResult& result
    );

    void printFlightLoopStarted();

    void printFlightSample(
        const ImuSample& sample,
        const FlightLoopIteration& iteration,
        const ImuHealthStatus& health
    );
}
