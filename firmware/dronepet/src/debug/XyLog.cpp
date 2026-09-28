#include "XyLog.h"

#include <SPIFFS.h>
#include <cmath>

namespace {

// The DronePet task/controller runs at 100 Hz. We log each new MTF sample
// rather than downsampling to the old 20 Hz diagnostic rate.
constexpr size_t MAX_SAMPLES = 1024;
constexpr uint8_t LOG_VERSION = 10;

constexpr const char* LOG_PATH = "/xylog.bin";
constexpr const char* TEMP_PATH = "/xylog.tmp";

#pragma pack(push, 1)
struct Sample {
    uint16_t timeMs;
    uint32_t sensorTimeMs;

    int16_t positionXmm;
    int16_t positionYmm;

    int16_t rawVelocityXmmps;
    int16_t rawVelocityYmmps;

    int16_t flowVelocityXmmps;
    int16_t flowVelocityYmmps;

    int16_t estimatedVelocityXmmps;
    int16_t estimatedVelocityYmmps;

    int16_t innovationXmmps;
    int16_t innovationYmmps;

    int16_t innovationRatioX1000;
    int16_t innovationRatioY1000;

    int16_t accelWorldX100;
    int16_t accelWorldY100;

    int16_t roll10000;
    int16_t pitch10000;

    int16_t unclampedRoll10000;
    int16_t unclampedPitch10000;

    int16_t rollITerm10000;
    int16_t pitchITerm10000;

    int16_t actualRoll100;
    int16_t actualPitch100;

    int16_t rollRate1000;
    int16_t pitchRate1000;

    int16_t rollRateSetpoint1000;
    int16_t pitchRateSetpoint1000;

    uint8_t flowAcceptedX;
    uint8_t flowAcceptedY;

    uint8_t flowQuality;
    uint8_t flowStatus;
};
#pragma pack(pop)

static_assert(sizeof(Sample) == 62);

#pragma pack(push, 1)
struct LogHeader {
    uint8_t version;
    uint16_t sampleCount;
};
#pragma pack(pop)

Sample samples[MAX_SAMPLES];
size_t sampleCount = 0;

uint32_t lastSensorTimeMs = 0;
bool haveLastSensorTime = false;

uint16_t toU16(float value, float scale) {
    int32_t scaled = lroundf(value * scale);
    if (scaled < 0) scaled = 0;
    if (scaled > 65535) scaled = 65535;
    return static_cast<uint16_t>(scaled);
}

int16_t toI16(float value, float scale) {
    int32_t scaled = lroundf(value * scale);
    if (scaled < -32768) scaled = -32768;
    if (scaled > 32767) scaled = 32767;
    return static_cast<int16_t>(scaled);
}

}

void DronePet::XyLog::beginRun() {
    sampleCount = 0;

    lastSensorTimeMs = 0;
    haveLastSensorTime = false;
}

void DronePet::XyLog::add(
    uint32_t elapsedMs,
    uint32_t sensorTimeMs,

    float positionXM,
    float positionYM,

    float rawVelocityXMps,
    float rawVelocityYMps,

    float flowVelocityXMps,
    float flowVelocityYMps,

    float estimatedVelocityXMps,
    float estimatedVelocityYMps,

    float innovationXMps,
    float innovationYMps,

    float innovationRatioX,
    float innovationRatioY,

    float accelWorldXMps2,
    float accelWorldYMps2,

    float rollCommand,
    float pitchCommand,

    float unclampedRollCommand,
    float unclampedPitchCommand,

    float rollITerm,
    float pitchITerm,

    float actualRollDeg,
    float actualPitchDeg,

    float rollRateRadS,
    float pitchRateRadS,

    float rollRateSetpointRadS,
    float pitchRateSetpointRadS,

    bool flowAcceptedX,
    bool flowAcceptedY,

    uint8_t flowQuality,
    uint8_t flowStatus
) {
   if (sampleCount >= MAX_SAMPLES) return;

    // Never log the same MTF packet twice.
    if (haveLastSensorTime && sensorTimeMs == lastSensorTimeMs) return;

    lastSensorTimeMs = sensorTimeMs;
    haveLastSensorTime = true;

    auto& sample = samples[sampleCount++];

    sample.timeMs = toU16(elapsedMs, 1.0f);
    sample.sensorTimeMs = sensorTimeMs;

    sample.positionXmm = toI16(positionXM, 1000.0f);
    sample.positionYmm = toI16(positionYM, 1000.0f);

    sample.rawVelocityXmmps = toI16(rawVelocityXMps, 1000.0f);
    sample.rawVelocityYmmps = toI16(rawVelocityYMps, 1000.0f);

    sample.flowVelocityXmmps = toI16(flowVelocityXMps, 1000.0f);
    sample.flowVelocityYmmps = toI16(flowVelocityYMps, 1000.0f);

    sample.estimatedVelocityXmmps = toI16(estimatedVelocityXMps, 1000.0f);
    sample.estimatedVelocityYmmps = toI16(estimatedVelocityYMps, 1000.0f);

    sample.innovationXmmps = toI16(innovationXMps, 1000.0f);
    sample.innovationYmmps = toI16(innovationYMps, 1000.0f);

    sample.innovationRatioX1000 = toI16(innovationRatioX, 1000.0f);

    sample.innovationRatioY1000 = toI16(innovationRatioY, 1000.0f);

    // 0.01 m/s² resolution, but allows ±327 m/s².
    sample.accelWorldX100 = toI16(accelWorldXMps2, 100.0f);
    sample.accelWorldY100 = toI16(accelWorldYMps2, 100.0f);

    sample.roll10000 = toI16(rollCommand, 10000.0f);
    sample.pitch10000 = toI16(pitchCommand, 10000.0f);

    sample.unclampedRoll10000 = toI16(unclampedRollCommand, 10000.0f);
    sample.unclampedPitch10000 = toI16(unclampedPitchCommand, 10000.0f);

    sample.rollITerm10000 = toI16(rollITerm, 10000.0f);
    sample.pitchITerm10000 = toI16(pitchITerm, 10000.0f);

    sample.actualRoll100 = toI16(actualRollDeg, 100.0f);
    sample.actualPitch100 = toI16(actualPitchDeg, 100.0f);

    sample.rollRate1000 = toI16(rollRateRadS, 1000.0f);
    sample.pitchRate1000 = toI16(pitchRateRadS, 1000.0f);

    sample.rollRateSetpoint1000 = toI16(rollRateSetpointRadS, 1000.0f);
    sample.pitchRateSetpoint1000 = toI16(pitchRateSetpointRadS, 1000.0f);

    sample.flowAcceptedX = flowAcceptedX ? 1 : 0;
    sample.flowAcceptedY = flowAcceptedY ? 1 : 0;

    sample.flowQuality = flowQuality;
    sample.flowStatus = flowStatus;
}

bool DronePet::XyLog::save() {
    if (sampleCount == 0) return false;

    SPIFFS.remove(TEMP_PATH);

    File file = SPIFFS.open(TEMP_PATH, FILE_WRITE);
    if (!file) return false;

    const LogHeader header{
        LOG_VERSION,
        static_cast<uint16_t>(sampleCount)
    };

    const size_t sampleBytes = sampleCount * sizeof(Sample);

    const bool headerOk =
        file.write(
            reinterpret_cast<const uint8_t*>(&header),
            sizeof(header)
        ) == sizeof(header);

    const bool samplesOk =
        file.write(
            reinterpret_cast<const uint8_t*>(samples),
            sampleBytes
        ) == sampleBytes;

    file.flush();
    file.close();

    if (!headerOk || !samplesOk) {
        SPIFFS.remove(TEMP_PATH);
        return false;
    }

    SPIFFS.remove(LOG_PATH);

    if (!SPIFFS.rename(TEMP_PATH, LOG_PATH)) {
        return false;
    }

    return true;
}

void DronePet::XyLog::printSaved(Stream& out) {
    File file = SPIFFS.open(LOG_PATH, FILE_READ);

    if (!file) {
        out.println("# NO_XY_LOG");
        return;
    }

    LogHeader header{};

    if (file.read(
            reinterpret_cast<uint8_t*>(&header),
            sizeof(header)
        ) != sizeof(header)) {
        file.close();
        out.println("# XY_LOG_READ_ERROR");
        return;
    }

    if (header.version != LOG_VERSION ||
        header.sampleCount == 0 ||
        header.sampleCount > MAX_SAMPLES) {
        file.close();
        out.println("# NO_XY_LOG");
        return;
    }

    const size_t bytes =
        header.sampleCount * sizeof(Sample);

    if (file.size() != sizeof(LogHeader) + bytes) {
        file.close();
        out.println("# XY_LOG_READ_ERROR");
        return;
    }

    const size_t read = file.read(
        reinterpret_cast<uint8_t*>(samples),
        bytes
    );

    file.close();

    if (read != bytes) {
        out.println("# XY_LOG_READ_ERROR");
        return;
    }

    const size_t count = header.sampleCount;

    out.println("# XY_LOG_BEGIN");
    out.println(
        "time_ms,sensor_time_ms,"
        "position_x_m,position_y_m,"
        "raw_velocity_x_mps,raw_velocity_y_mps,"
        "flow_velocity_x_mps,flow_velocity_y_mps,"
        "estimated_velocity_x_mps,estimated_velocity_y_mps,"
        "innovation_x_mps,innovation_y_mps,"
        "innovation_ratio_x,innovation_ratio_y,"
        "accel_world_x_mps2,accel_world_y_mps2,"
        "roll_cmd,pitch_cmd,"
        "unclamped_roll_cmd,unclamped_pitch_cmd,"
        "roll_i_term,pitch_i_term,"
        "actual_roll_deg,actual_pitch_deg,"
        "roll_rate_rad_s,pitch_rate_rad_s,"
        "roll_rate_setpoint_rad_s,pitch_rate_setpoint_rad_s,"
        "flow_accepted_x,flow_accepted_y,"
        "flow_quality,flow_status"
    );
    for (size_t i = 0; i < count; i++) {
        const auto& sample = samples[i];

        out.printf(
            "%u,%lu,"
            "%.3f,%.3f,"  // position
            "%.3f,%.3f,"  // raw velocity
            "%.3f,%.3f,"  // flow velocity
            "%.3f,%.3f,"  // estimated velocity
            "%.3f,%.3f,"  // innovation
            "%.3f,%.3f,"  // innovation ratio
            "%.3f,%.3f,"  // world acceleration
            "%.4f,%.4f,"  // roll/pitch command
            "%.4f,%.4f,"  // unclamped command
            "%.4f,%.4f,"  // I terms
            "%.2f,%.2f,"  // actual attitude
            "%.3f,%.3f,"  // rates
            "%.3f,%.3f,"  // rate setpoints
            "%u,%u,%u,%u\n",
            sample.timeMs,
            static_cast<unsigned long>(sample.sensorTimeMs),

            sample.positionXmm / 1000.0f,
            sample.positionYmm / 1000.0f,

            sample.rawVelocityXmmps / 1000.0f,
            sample.rawVelocityYmmps / 1000.0f,

            sample.flowVelocityXmmps / 1000.0f,
            sample.flowVelocityYmmps / 1000.0f,

            sample.estimatedVelocityXmmps / 1000.0f,
            sample.estimatedVelocityYmmps / 1000.0f,

            sample.innovationXmmps / 1000.0f,
            sample.innovationYmmps / 1000.0f,

            sample.innovationRatioX1000 / 1000.0f,
            sample.innovationRatioY1000 / 1000.0f,

            sample.accelWorldX100 / 100.0f,
            sample.accelWorldY100 / 100.0f,

            sample.roll10000 / 10000.0f,
            sample.pitch10000 / 10000.0f,

            sample.unclampedRoll10000 / 10000.0f,
            sample.unclampedPitch10000 / 10000.0f,

            sample.rollITerm10000 / 10000.0f,
            sample.pitchITerm10000 / 10000.0f,

            sample.actualRoll100 / 100.0f,
            sample.actualPitch100 / 100.0f,

            sample.rollRate1000 / 1000.0f,
            sample.pitchRate1000 / 1000.0f,

            sample.rollRateSetpoint1000 / 1000.0f,
            sample.pitchRateSetpoint1000 / 1000.0f,

            sample.flowAcceptedX,
            sample.flowAcceptedY,

            sample.flowQuality,
            sample.flowStatus
        );
    }

    out.println("# XY_LOG_END");
}