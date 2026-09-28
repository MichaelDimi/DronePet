#include "AltitudeLog.h"

#include <SPIFFS.h>
#include <cmath>

namespace {

constexpr uint32_t LOG_PERIOD_MS = 50; // 20 Hz
constexpr size_t MAX_SAMPLES = 320;
constexpr uint8_t LOG_VERSION = 4;

constexpr const char* LOG_PATH = "/altlog.bin";
constexpr const char* TEMP_PATH = "/altlog.tmp";

struct Sample {
    uint16_t timeMs;

    uint16_t targetMm;
    uint16_t altitudeMm;
    uint16_t filteredAltitude01Mm;

    int16_t errorMm;

    int16_t pTerm100000;
    int16_t iTerm100000;
    int16_t dTerm100000;
    int16_t correction100000;

    uint16_t throttle10000;

    uint8_t strength;
    uint8_t precision;
    uint8_t tofStatus;
};

static_assert(sizeof(Sample) == 24);

#pragma pack(push, 1)
struct LogHeader {
    uint8_t version;
    uint16_t sampleCount;
};
#pragma pack(pop)

Sample samples[MAX_SAMPLES];

size_t sampleCount = 0;
uint32_t lastSampleMs = 0;
bool haveLastSample = false;

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

void DronePet::AltitudeLog::beginRun() {
    sampleCount = 0;
    lastSampleMs = 0;
    haveLastSample = false;
}

void DronePet::AltitudeLog::add(
    uint32_t elapsedMs,
    float targetAltitudeM,
    float altitudeM,
    float filteredAltitudeM,
    float errorM,
    float pTerm,
    float iTerm,
    float dTerm,
    float correction,
    float throttle,
    uint8_t strength,
    uint8_t precision,
    uint8_t tofStatus
) {
    if (sampleCount >= MAX_SAMPLES) return;

    if (haveLastSample && elapsedMs - lastSampleMs < LOG_PERIOD_MS) return;

    lastSampleMs = elapsedMs;
    haveLastSample = true;

    auto& sample = samples[sampleCount++];

    sample.timeMs = toU16(elapsedMs, 1.0f);

    sample.targetMm = toU16(targetAltitudeM, 1000.0f);
    sample.altitudeMm = toU16(altitudeM, 1000.0f);
    sample.filteredAltitude01Mm = toU16(filteredAltitudeM, 10000.0f);

    sample.errorMm = toI16(errorM, 1000.0f);

    sample.pTerm100000 = toI16(pTerm, 100000.0f);
    sample.iTerm100000 = toI16(iTerm, 100000.0f);
    sample.dTerm100000 = toI16(dTerm, 100000.0f);
    sample.correction100000 = toI16(correction, 100000.0f);

    sample.throttle10000 = toU16(throttle, 10000.0f);

    sample.strength = strength;
    sample.precision = precision;
    sample.tofStatus = tofStatus;
}

bool DronePet::AltitudeLog::save() {
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

void DronePet::AltitudeLog::printSaved(Stream& out) {
    File file = SPIFFS.open(LOG_PATH, FILE_READ);

    if (!file) {
        out.println("# NO_ALTITUDE_LOG");
        return;
    }

    LogHeader header{};

    if (file.read(
            reinterpret_cast<uint8_t*>(&header),
            sizeof(header)
        ) != sizeof(header)) {
        file.close();
        out.println("# ALTITUDE_LOG_READ_ERROR");
        return;
    }

    if (header.version != LOG_VERSION ||
        header.sampleCount == 0 ||
        header.sampleCount > MAX_SAMPLES) {
        file.close();
        out.println("# NO_ALTITUDE_LOG");
        return;
    }

    const size_t bytes =
        header.sampleCount * sizeof(Sample);

    if (file.size() != sizeof(LogHeader) + bytes) {
        file.close();
        out.println("# ALTITUDE_LOG_READ_ERROR");
        return;
    }

    const size_t read = file.read(
        reinterpret_cast<uint8_t*>(samples),
        bytes
    );

    file.close();

    if (read != bytes) {
        out.println("# ALTITUDE_LOG_READ_ERROR");
        return;
    }

    const size_t count = header.sampleCount;

    out.println("# ALTITUDE_LOG_BEGIN");
    out.println(
        "time_ms,target_m,altitude_m,filtered_altitude_m,"
        "error_m,p_term,i_term,d_term,correction,throttle,"
        "strength,precision,tof_status"
    );

    for (size_t i = 0; i < count; i++) {
        const auto& sample = samples[i];

        out.printf(
            "%u,%.3f,%.3f,%.4f,%.3f,%.5f,%.5f,%.5f,%.5f,%.4f,%u,%u,%u\n",
            sample.timeMs,
            sample.targetMm / 1000.0f,
            sample.altitudeMm / 1000.0f,
            sample.filteredAltitude01Mm / 10000.0f,
            sample.errorMm / 1000.0f,
            sample.pTerm100000 / 100000.0f,
            sample.iTerm100000 / 100000.0f,
            sample.dTerm100000 / 100000.0f,
            sample.correction100000 / 100000.0f,
            sample.throttle10000 / 10000.0f,
            sample.strength,
            sample.precision,
            sample.tofStatus
        );
    }

    out.println("# ALTITUDE_LOG_END");
}