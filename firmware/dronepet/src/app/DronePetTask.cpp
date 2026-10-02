#include <Arduino.h>
#include <cmath>

#include "DronePetTask.h"

#include "../sensors/Mtf01.h"
#include "../board/StatusLed.h"

#include "../controller/AltitudeController.h"
#include "../estimation/AltitudeEstimator.h"
#include "../controller/XyVelocityController.h"
#include "../estimation/VelocityEstimator.h"

#include "../debug/AltitudeLog.h"
#include "../debug/XyLog.h"

#include "../flight/EspFcRuntime.h"
#include "../flight/FlightCommandMailbox.h"



namespace {

// Testing constants
constexpr uint32_t TASK_STACK_SIZE = 8192;
constexpr uint32_t TEST_START_DELAY_MS = 2000;

// Holding
constexpr uint32_t HOLD_DURATION_MS = 4000;

// Landing
constexpr float LAND_DESCENT_RATE_MPS = 0.25f;
constexpr float VERTICAL_DESCENT_RATE_MPS = 0.15f;

constexpr float XY_DISABLE_ALTITUDE_M = 0.15f;
constexpr float TOF_CONTROL_MIN_ALTITUDE_M = 0.10f;

constexpr float TOUCHDOWN_THROTTLE = 0.18;
constexpr uint32_t TOUCHDOWN_TIMEOUT_MS = 2000;

// Safety
constexpr float MAX_TILT_DEG = 25.0f;
constexpr float MIN_ALTITUDE_M = XY_DISABLE_ALTITUDE_M;

enum class TestPhase {
    Waiting,
    Arming,
    Holding,
    Landing,
    Done,
    Aborted
};

enum class LandingStage {
    ControlledDescent,  // XY + altitude control
    VerticalDescent,    // altitude control only
    Touchdown           // later: open-loop final descent + touchdown detection
};

struct TaskContext {
    DronePet::AltitudeController altitudeController;
    DronePet::AltitudeEstimator altitudeEstimator;
    DronePet::XyVelocityController xyVelocityController;
    DronePet::VelocityEstimator velocityEstimator;

    TestPhase phase = TestPhase::Waiting;
    LandingStage landingStage = LandingStage::ControlledDescent;
    
    DronePet::XyControlOutput xyOutput{};

    float targetAltitudeM = 0.0f;

    uint32_t taskStartedMs = 0;
    uint32_t flightStartedMs = 0;
    uint32_t touchdownStartedMs = 0;

    uint32_t landingLastUpdateMs = 0; // dt for landing

    bool logStarted = false;
    bool logSaveAttempted = false;
    bool serialWasConnected = false;
};

void setPhase(TaskContext& ctx, TestPhase phase) {
    ctx.phase = phase;

    switch (phase) {
        case TestPhase::Holding:
            DronePet::StatusLed::green();
            break;

        case TestPhase::Aborted:
            DronePet::StatusLed::red();
            break;

        case TestPhase::Landing:
            DronePet::StatusLed::blue();
            break;

        default:
            DronePet::StatusLed::off();
            break;
    }
}

void setLandingStage(TaskContext& ctx, LandingStage stage, uint32_t nowMs) {
    ctx.landingStage = stage;

    switch (stage) {
        case LandingStage::ControlledDescent:
            break;

        case LandingStage::VerticalDescent:
            ctx.xyVelocityController.reset();
            ctx.xyOutput = {};
            break;

        case LandingStage::Touchdown:
            ctx.altitudeController.reset();
            ctx.touchdownStartedMs = nowMs;
            break;
    }
}

bool isFinished(TestPhase phase) {
    return phase == TestPhase::Done || phase == TestPhase::Aborted;
}

void initializeTask(TaskContext& ctx) {
    DronePet::StatusLed::begin();
    DronePet::Mtf01::begin();

    ctx.altitudeController.begin();
    ctx.xyVelocityController.begin();

    ctx.taskStartedMs = millis();
    setPhase(ctx, TestPhase::Waiting);
}

void handleSerialConnection(TaskContext& ctx, bool serialConnected) {
    if (serialConnected && !ctx.serialWasConnected &&
        ctx.phase != TestPhase::Arming && 
        ctx.phase != TestPhase::Holding &&
        ctx.phase != TestPhase::Landing) {
        DronePet::AltitudeLog::printSaved(Serial);
        DronePet::XyLog::printSaved(Serial);
    }

    ctx.serialWasConnected = serialConnected;
}

void abortTest(TaskContext& ctx) {
    setPhase(ctx, TestPhase::Aborted);
}

void updateEstimators(
    TaskContext& ctx,
    bool newMtfSample,
    const Espfc::RuntimeStatus& fcStatus
) {
    if (!newMtfSample) return;

    const auto& sample = DronePet::Mtf01::latestSample();

    ctx.altitudeEstimator.update(
        sample.rangeM,
        fcStatus.rollDeg * DEG_TO_RAD,
        fcStatus.pitchDeg * DEG_TO_RAD
    );

    ctx.velocityEstimator.update(
        sample.sensorTimeMs,
        sample.flowVelocityXAt1mMps,
        sample.flowVelocityYAt1mMps,
        sample.rangeM,
        fcStatus.rollRateRadS,
        fcStatus.pitchRateRadS,
        fcStatus.yawRad
    );
}

void beginHold(
    TaskContext& ctx,
    uint32_t sensorTimeMs
) {
    ctx.targetAltitudeM = ctx.altitudeEstimator.state().altitudeM;

    ctx.altitudeController.reset();

    ctx.velocityEstimator.reset(sensorTimeMs);
    ctx.xyVelocityController.reset();
    ctx.xyOutput = {};

    DronePet::AltitudeLog::beginRun();
    DronePet::XyLog::beginRun();

    ctx.logStarted = true;
    ctx.logSaveAttempted = false;

    ctx.flightStartedMs = millis();

    setPhase(ctx, TestPhase::Holding);
}

void beginLanding(TaskContext& ctx) {
    const uint32_t nowMs = millis();

    ctx.landingLastUpdateMs = nowMs;
    setLandingStage(ctx, LandingStage::ControlledDescent, nowMs);
    setPhase(ctx, TestPhase::Landing);
    // setPhase(ctx, TestPhase::Done);
}

bool flightIsSafe(const Espfc::RuntimeStatus& fcStatus) {
    if (!fcStatus.ready || !fcStatus.armed) return false;

    if (std::fabs(fcStatus.rollDeg) > MAX_TILT_DEG ||
        std::fabs(fcStatus.pitchDeg) > MAX_TILT_DEG) return false;

    return DronePet::Mtf01::fresh();
}

bool holdingIsSafe(
    const Espfc::RuntimeStatus& fcStatus,
    float altitudeM
) {
    if (!flightIsSafe(fcStatus)) return false;
    return altitudeM >= MIN_ALTITUDE_M;
}

void updateXyControl(TaskContext& ctx) {
    const auto& velocity = ctx.velocityEstimator.state();

    ctx.xyOutput = ctx.xyVelocityController.update(
        0.0f,
        0.0f,
        velocity.velocityXMps,
        velocity.velocityYMps
    );
}

void logAltitude(
    const TaskContext& ctx,
    uint32_t elapsedMs,
    float altitudeM,
    const DronePet::MtfSample& sample,
    const DronePet::PilotCommand& command
) {
    const auto& debug = ctx.altitudeController.debug();

    DronePet::AltitudeLog::add(
        elapsedMs,
        ctx.targetAltitudeM,
        altitudeM,
        debug.filteredAltitudeM,
        debug.errorM,
        debug.pTerm,
        debug.iTerm,
        debug.dTerm,
        debug.correction,
        command.throttle,
        sample.strength,
        sample.precision,
        sample.tofStatus
    );
}

void logXy(
    const TaskContext& ctx,
    uint32_t elapsedMs,
    const DronePet::MtfSample& sample,
    const Espfc::RuntimeStatus& fcStatus,
    const DronePet::PilotCommand& command
) {
    const auto& velocityEstimate = ctx.velocityEstimator.state();

    DronePet::XyLog::add(
        elapsedMs,
        sample.sensorTimeMs,
        velocityEstimate.rawVelocityXMps,
        velocityEstimate.rawVelocityYMps,
        velocityEstimate.compensatedVelocityXMps,
        velocityEstimate.compensatedVelocityYMps,
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
        ctx.xyOutput.unclampedRollCommand,
        ctx.xyOutput.unclampedPitchCommand,
        ctx.xyOutput.rollITerm,
        ctx.xyOutput.pitchITerm,
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

void handleWaiting(
    TaskContext& ctx,
    const Espfc::RuntimeStatus& fcStatus,
    bool serialConnected
) {
    // Keep the drone idle while USB/Serial is connected so logs can be retrieved safely.
    if (serialConnected) return;

    // Start arming once the startup delay has passed and the flight controller is ready.
    if (millis() - ctx.taskStartedMs >= TEST_START_DELAY_MS && fcStatus.ready) {
        setPhase(ctx, TestPhase::Arming);
    }
}

void handleArming(
    TaskContext& ctx,
    const Espfc::RuntimeStatus& fcStatus,
    DronePet::PilotCommand& command
) {
    command.armed = true;

    // Abort if the flight controller becomes unavailable while trying to arm.
    if (!fcStatus.ready) {
        abortTest(ctx);
        return;
    }

    // Wait here until the flight controller confirms that it is armed.
    if (!fcStatus.armed) return;

    // Require a valid starting altitude before beginning the hold test.
    if (!DronePet::Mtf01::fresh()) {
        abortTest(ctx);
        return;
    }

    const auto& sample = DronePet::Mtf01::latestSample();
    const float altitudeM = ctx.altitudeEstimator.state().altitudeM;

    if (altitudeM < MIN_ALTITUDE_M) {
        abortTest(ctx);
        return;
    }

    // Initialize the controllers, estimators, and logs from the current hover state.
    beginHold(ctx, sample.sensorTimeMs);
}

void handleHolding(
    TaskContext& ctx,
    const Espfc::RuntimeStatus& fcStatus,
    bool newMtfSample,
    DronePet::PilotCommand& command
) {
    const auto& sample = DronePet::Mtf01::latestSample();
    const float altitudeM = ctx.altitudeEstimator.state().altitudeM;

    if (!holdingIsSafe(fcStatus, altitudeM)) {
        abortTest(ctx);
        return;
    }

    command.armed = true;

    const uint32_t elapsedMs = millis() - ctx.flightStartedMs;

    // Update horizontal velocity hold whenever a new optical-flow sample arrives.
    if (newMtfSample) { updateXyControl(ctx); }

    command.roll = ctx.xyOutput.roll;
    command.pitch = ctx.xyOutput.pitch;

    // Continuously maintain the altitude captured at the start of the test.
    command.throttle = ctx.altitudeController.update(ctx.targetAltitudeM, altitudeM);

    // Record control and sensor data for post-flight analysis.
    logAltitude(ctx, elapsedMs, altitudeM, sample, command);
    if (newMtfSample) logXy(ctx, elapsedMs, sample, fcStatus, command);

    // Go to landing after the configured hold duration.
    if (elapsedMs >= HOLD_DURATION_MS) beginLanding(ctx);
}

bool touchdownDetected() {
    return false;
}

void handleLanding(
    TaskContext& ctx,
    const Espfc::RuntimeStatus& fcStatus,
    bool newMtfSample,
    DronePet::PilotCommand& command
) {
    if (!flightIsSafe(fcStatus)) {
        abortTest(ctx);
        return;
    }
    
    command.armed = true;

    const uint32_t nowMs = millis();
    const uint32_t elapsedMs = nowMs - ctx.flightStartedMs;
    const float dt = (nowMs - ctx.landingLastUpdateMs) / 1000.0f;
    ctx.landingLastUpdateMs = nowMs;

    const auto& sample = DronePet::Mtf01::latestSample();
    const float altitudeM = ctx.altitudeEstimator.state().altitudeM;

    switch (ctx.landingStage) {
        case LandingStage::ControlledDescent:
            ctx.targetAltitudeM -= LAND_DESCENT_RATE_MPS * dt;

            if (newMtfSample) updateXyControl(ctx);

            command.roll = ctx.xyOutput.roll;
            command.pitch = ctx.xyOutput.pitch;
            command.throttle = ctx.altitudeController.update(ctx.targetAltitudeM, altitudeM);

            // Logging
            logAltitude(ctx, elapsedMs, altitudeM, sample, command);
            if (newMtfSample) logXy(ctx, elapsedMs, sample, fcStatus, command);

            if (altitudeM <= XY_DISABLE_ALTITUDE_M) {
                setLandingStage(ctx, LandingStage::VerticalDescent, nowMs);
            }
            break;

        case LandingStage::VerticalDescent:
            ctx.targetAltitudeM -= VERTICAL_DESCENT_RATE_MPS * dt;

            command.roll = 0.0f;
            command.pitch = 0.0f;
            command.throttle = ctx.altitudeController.update(ctx.targetAltitudeM, altitudeM);

            logAltitude(ctx, elapsedMs, altitudeM, sample, command);
            if (newMtfSample) logXy(ctx, elapsedMs, sample, fcStatus, command);

            if (altitudeM <= TOF_CONTROL_MIN_ALTITUDE_M) {
                setLandingStage(ctx, LandingStage::Touchdown, nowMs);
            }
            break;

        case LandingStage::Touchdown:
            command.roll = 0.0f;
            command.pitch = 0.0f;
            command.throttle = TOUCHDOWN_THROTTLE;

            logAltitude(ctx, elapsedMs, altitudeM, sample, command);
            if (newMtfSample) logXy(ctx, elapsedMs, sample, fcStatus, command);

            if (touchdownDetected() ||
                nowMs - ctx.touchdownStartedMs >= TOUCHDOWN_TIMEOUT_MS) {
                setPhase(ctx, TestPhase::Done);
            }
            break;
    }
}

void handlePhase(
    TaskContext& ctx,
    const Espfc::RuntimeStatus& fcStatus,
    bool serialConnected,
    bool newMtfSample,
    DronePet::PilotCommand& command
) {
    switch (ctx.phase) {
        case TestPhase::Waiting:
            handleWaiting(ctx, fcStatus, serialConnected);
            break;

        case TestPhase::Arming:
            handleArming(ctx, fcStatus, command);
            break;

        case TestPhase::Holding:
            handleHolding(ctx, fcStatus, newMtfSample, command);
            break;

        case TestPhase::Landing:
            handleLanding(ctx, fcStatus, newMtfSample, command);
            break;

        case TestPhase::Done:
        case TestPhase::Aborted:
            break;
    }
}

void publishCommand(TaskContext& ctx, DronePet::PilotCommand& command) {
    const bool testFinished = isFinished(ctx.phase);

    if (testFinished) {
        command.armed = false;
        command.throttle = 0.0f;
    }

    DronePet::FlightCommandMailbox::publish(command);

    // Save only after the motor-off command has already been published.
    if (testFinished && ctx.logStarted && !ctx.logSaveAttempted) {
        ctx.logSaveAttempted = true;
        DronePet::AltitudeLog::save();
        DronePet::XyLog::save();
    }
}

void run(void*) {
    TaskContext ctx;
    initializeTask(ctx);

    while (true) {
        const bool newMtfSample = DronePet::Mtf01::update();
        const bool serialConnected = static_cast<bool>(Serial);
        handleSerialConnection(ctx, serialConnected);

        const auto fcStatus = EspFcRuntime::status();

        updateEstimators(ctx, newMtfSample, fcStatus);

        DronePet::PilotCommand command{};
        command.angleMode = true;

        handlePhase(
            ctx,
            fcStatus,
            serialConnected,
            newMtfSample,
            command
        );

        // Publish the resulting command and handle one-time shutdown work.
        publishCommand(ctx, command);

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
