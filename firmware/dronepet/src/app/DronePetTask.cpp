#include <Arduino.h>

#include "DronePetTask.h"

#include "../board/BoardPins.h"
#include "../board/StatusLed.h"
#include "../controller/AltitudeController.h"
#include "../flight/EspFcRuntime.h"
#include "../flight/FlightCommandMailbox.h"
#include "../sensors/Mtf01.h"


namespace {

constexpr uint32_t TEST_START_DELAY_MS = 2000;
constexpr uint32_t HOLD_DURATION_MS = 10000;
constexpr uint32_t MIN_ALTITUDE_MM = 10;

constexpr float TEST_THROTTLE = 0.20f;
constexpr float MAX_TILT_DEG = 45.0f;

enum class TestPhase {
    Waiting,
    Arming,
    Holding,
    Done,
    Aborted
};

bool readAltitude(float& altitudeM) {
    if (!DronePet::Mtf01::fresh()) return false;

    const auto& sample = DronePet::Mtf01::latestSample();
    if (sample.distanceMm < MIN_ALTITUDE_MM) return false;

    altitudeM = sample.distanceMm / 1000.0f;
    return true;
}

void setLed(TestPhase phase) {
    switch (phase) {
        case TestPhase::Arming:
            DronePet::StatusLed::blue();
            break;

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


void run(void* parameter) {
    DronePet::StatusLed::begin();
    // DronePet::Mtf01::begin();

    // DronePet::AltitudeController altitudeController;
    // altitudeController.begin();

    TestPhase phase = TestPhase::Waiting;
    setLed(phase);

    uint32_t readySinceMs = 0;
    uint32_t holdStartedMs = 0;

    // float targetAltitudeM = 0.0f;
    // float lastThrottle = 0.0f;

    while (true) {
        // const bool newMtfSample = DronePet::Mtf01::update();
        const auto fcStatus = EspFcRuntime::status();

        DronePet::PilotCommand command;
        command.angleMode = true;

        switch (phase) {
            case TestPhase::Waiting: {
                command.armed = false;

                if (!fcStatus.ready) {
                    readySinceMs = 0;
                    break;
                }

                if (readySinceMs == 0) {
                    readySinceMs = millis();
                    break;
                }

                if (millis() - readySinceMs >= TEST_START_DELAY_MS) {
                    phase = TestPhase::Arming;
                    setLed(phase);
                }

                break;

            }

            case TestPhase::Arming: {
                command.armed = true;
                command.throttle = 0.0f;

                if (!fcStatus.ready) {
                    phase = TestPhase::Aborted;
                    setLed(phase);
                    break;
                }

                if (fcStatus.armed) {
                    holdStartedMs = millis();

                    phase = TestPhase::Holding;
                    setLed(phase);
                }

                break;
            }

            case TestPhase::Holding: {
                command.armed = true;
                command.throttle = TEST_THROTTLE;

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

                if (millis() - holdStartedMs >= HOLD_DURATION_MS) {
                    phase = TestPhase::Done;
                    setLed(phase);
                }

                break;
            }

            case TestPhase::Done:
            case TestPhase::Aborted:
                command.armed = false;
                command.throttle = 0.0f;
                break;
        }

        DronePet::FlightCommandMailbox::publish(command);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
}


void DronePetTask::start() {
    DronePet::PilotCommand command;

    command.armed = false;
    command.angleMode = true;

    DronePet::FlightCommandMailbox::publish(command);

    xTaskCreatePinnedToCore(run, "DronePet", 4096, nullptr, 1, nullptr, 1);
}