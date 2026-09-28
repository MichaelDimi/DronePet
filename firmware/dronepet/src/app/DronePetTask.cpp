#include <Arduino.h>

#include "DronePetTask.h"

#include "../board/StatusLed.h"

#include "../estimation/XyStateEstimator.h"
#include "../estimation/VelocityEstimator.h"

#include "../controller/AltitudeController.h"
#include "../controller/XyVelocityController.h"

#include "../debug/AltitudeLog.h"
#include "../debug/XyLog.h"

#include "../flight/EspFcRuntime.h"
#include "../flight/FlightCommandMailbox.h"

#include "../sensors/Mtf01.h"

#include <cmath>

namespace {

constexpr uint32_t TASK_STACK_SIZE = 8192;

constexpr uint32_t TEST_START_DELAY_MS = 2000;
constexpr uint32_t HOLD_DURATION_MS = 10000;

constexpr float MAX_TILT_DEG = 25.0f;
constexpr float MIN_ALTITUDE_M = 0.10f;

enum class TestPhase {
    Waiting,
    Arming,
    Holding,
    Done,
    Aborted
};

void setLed(TestPhase phase) {
    switch (phase) {
        case TestPhase::Holding:
            DronePet::StatusLed::green();
            break;

        case TestPhase::Aborted:
            DronePet::StatusLed::red();
            break;

        default:
            DronePet::StatusLed::off();
            break;
    }
}

void run(void*) {
    DronePet::StatusLed::begin();
    DronePet::Mtf01::begin();

    DronePet::AltitudeController altitudeController;
    altitudeController.begin();

    DronePet::XyVelocityController xyVelocityController;
    xyVelocityController.begin();

    DronePet::XyStateEstimator xyStateEstimator;
    DronePet::VelocityEstimator velocityEstimator;

    TestPhase phase = TestPhase::Waiting;
    setLed(phase);

    const uint32_t taskStartedMs = millis();
    uint32_t holdStartedMs = 0;

    float targetAltitudeM = 0.0f;

    DronePet::XyControlOutput xyOutput{};

    bool logStarted = false;
    bool logSaveAttempted = false;
    bool serialWasConnected = false;

    while (true) {
        const bool newMtfSample = DronePet::Mtf01::update();

        const bool serialConnected = static_cast<bool>(Serial);

        if (serialConnected && !serialWasConnected &&
            phase != TestPhase::Arming && phase != TestPhase::Holding) {
            DronePet::AltitudeLog::printSaved(Serial);
            DronePet::XyLog::printSaved(Serial);
        }

        serialWasConnected = serialConnected;

        const auto fcStatus = EspFcRuntime::status();

        DronePet::PilotCommand command{};
        command.angleMode = true;

        switch (phase) {
            case TestPhase::Waiting:
                if (serialConnected) break;

                if (millis() - taskStartedMs >= TEST_START_DELAY_MS && fcStatus.ready) {
                    phase = TestPhase::Arming;
                    setLed(phase);
                }
                break;

            case TestPhase::Arming:
                command.armed = true;

                if (!fcStatus.ready) {
                    phase = TestPhase::Aborted;
                    setLed(phase);
                    break;
                }

                if (fcStatus.armed) {
                    if (!DronePet::Mtf01::fresh()) {
                        phase = TestPhase::Aborted;
                        setLed(phase);
                        break;
                    }

                    const auto& sample = DronePet::Mtf01::latestSample();

                    if (sample.distanceMm < 100) {
                        phase = TestPhase::Aborted;
                        setLed(phase);
                        break;
                    }

                    targetAltitudeM = sample.distanceMm / 1000.0f;

                    altitudeController.reset();

                    xyStateEstimator.reset(sample.sensorTimeMs);
                    velocityEstimator.reset(sample.sensorTimeMs);

                    xyOutput = {};

                    xyVelocityController.reset();

                    DronePet::AltitudeLog::beginRun();
                    DronePet::XyLog::beginRun();
                    logStarted = true;
                    logSaveAttempted = false;

                    holdStartedMs = millis();

                    phase = TestPhase::Holding;
                    setLed(phase);
                }
                break;

            case TestPhase::Holding: {
                command.armed = true;

                if (!fcStatus.ready || !fcStatus.armed) {
                    phase = TestPhase::Aborted;
                    setLed(phase);
                    break;
                }

                if (std::fabs(fcStatus.rollDeg) > MAX_TILT_DEG ||
                    std::fabs(fcStatus.pitchDeg) > MAX_TILT_DEG) {
                    phase = TestPhase::Aborted;
                    setLed(phase);
                    break;
                }

                if (!DronePet::Mtf01::fresh()) {
                    phase = TestPhase::Aborted;
                    setLed(phase);
                    break;
                }

                const auto& sample = DronePet::Mtf01::latestSample();
                const float altitudeM = sample.distanceMm / 1000.0f;

                if (altitudeM < MIN_ALTITUDE_M) {
                    phase = TestPhase::Aborted;
                    setLed(phase);
                    break;
                }

                if (newMtfSample) {
                    const auto& xyState = xyStateEstimator.update(
                        sample.sensorTimeMs,
                        sample.velocityXMps,
                        sample.velocityYMps,
                        altitudeM,
                        fcStatus.rollRateRadS,
                        fcStatus.pitchRateRadS
                    );

                    const auto& velocityEstimate =
                        velocityEstimator.update(
                            sample.sensorTimeMs,
                            xyState.velocityXMps,
                            xyState.velocityYMps,
                            fcStatus.yawRad
                        );

                    xyOutput = xyVelocityController.update(
                        0.0f,
                        0.0f,
                        velocityEstimate.velocityXMps,
                        velocityEstimate.velocityYMps
                    );
                }

                command.roll = xyOutput.roll;
                command.pitch = xyOutput.pitch;

                const uint32_t elapsedMs = millis() - holdStartedMs;

                command.throttle = altitudeController.update(targetAltitudeM, altitudeM);
                const auto& altitudeDebug = altitudeController.debug();

                DronePet::AltitudeLog::add(
                    elapsedMs,
                    targetAltitudeM,
                    altitudeM,
                    altitudeDebug.filteredAltitudeM,
                    altitudeDebug.errorM,
                    altitudeDebug.pTerm,
                    altitudeDebug.iTerm,
                    altitudeDebug.dTerm,
                    altitudeDebug.correction,
                    command.throttle,
                    sample.strength,
                    sample.precision,
                    sample.tofStatus
                );

                const auto& xyState = xyStateEstimator.state();
                const auto& velocityEstimate = velocityEstimator.state();

                // XyLog internally rejects duplicate sensorTimeMs values, so the
                // resulting log follows fresh MTF packets instead of the old 20 Hz
                // timer. With the current 10 ms task loop this is at most ~100 Hz.
                if (newMtfSample) {
                    DronePet::XyLog::add(
                        elapsedMs,
                        sample.sensorTimeMs,
                        xyState.positionXM,
                        xyState.positionYM,
                        sample.velocityXMps,
                        sample.velocityYMps,
                        xyState.velocityXMps,
                        xyState.velocityYMps,
                        velocityEstimate.velocityXMps,
                        velocityEstimate.velocityYMps,
                        velocityEstimate.innovationXMps,
                        velocityEstimate.innovationYMps,
                        velocityEstimate.innovationRatioX,
                        velocityEstimate.innovationRatioY,
                        fcStatus.accelWorldXMps2,
                        fcStatus.accelWorldYMps2,
                        command.roll,
                        command.pitch,
                        xyOutput.unclampedRollCommand,
                        xyOutput.unclampedPitchCommand,
                        xyOutput.rollITerm,
                        xyOutput.pitchITerm,
                        fcStatus.rollDeg,
                        fcStatus.pitchDeg,
                        fcStatus.rollRateRadS,
                        fcStatus.pitchRateRadS,
                        fcStatus.rollRateSetpointRadS,
                        fcStatus.pitchRateSetpointRadS,
                        velocityEstimate.flowAcceptedX,
                        velocityEstimate.flowAcceptedY,
                        sample.flowQuality,
                        sample.flowStatus
                    );
                }

                if (elapsedMs >= HOLD_DURATION_MS) {
                    phase = TestPhase::Done;
                    setLed(phase);
                }

                break;
            }

            case TestPhase::Done:
            case TestPhase::Aborted:
                break;
        }

        const bool finished = phase == TestPhase::Done || phase == TestPhase::Aborted;

        if (finished) {
            command.armed = false;
            command.throttle = 0.0f;
        }

        DronePet::FlightCommandMailbox::publish(command);

        // Write only after the motor-off command has already been published.
        if (finished && logStarted && !logSaveAttempted) {
            logSaveAttempted = true;
            DronePet::AltitudeLog::save();
            DronePet::XyLog::save();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

}

void DronePetTask::start() {
    DronePet::PilotCommand command{};
    command.angleMode = true;

    DronePet::FlightCommandMailbox::publish(command);

    xTaskCreatePinnedToCore(run, "DronePet", TASK_STACK_SIZE, nullptr, 1, nullptr, 1);
}