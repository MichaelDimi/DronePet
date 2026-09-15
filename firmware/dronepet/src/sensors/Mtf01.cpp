
#include "Mtf01.h"

#include "../board/BoardPins.h"

#include <cstring>


namespace {

constexpr uint32_t MTF_BAUD = 115200;

constexpr uint8_t MICOLINK_HEAD = 0xEF;
constexpr uint8_t RANGE_SENSOR_MSG_ID = 0x51;

constexpr size_t MAX_PAYLOAD_LEN = 64;

constexpr uint32_t FRESH_TIMEOUT_US = 100000;


#pragma pack(push, 1)

struct RangeSensorPayload {
    uint32_t timeMs;
    uint32_t distanceMm;

    uint8_t strength;
    uint8_t precision;
    uint8_t tofStatus;
    uint8_t reserved1;

    int16_t flowVelX;
    int16_t flowVelY;

    uint8_t flowQuality;
    uint8_t flowStatus;

    uint16_t reserved2;
};

#pragma pack(pop)

static_assert(
    sizeof(RangeSensorPayload) == 20
);


enum class ParseState : uint8_t {
    Head,
    DeviceId,
    SystemId,
    MessageId,
    Sequence,
    Length,
    Payload,
    Checksum,
};


struct Parser {
    ParseState state = ParseState::Head;

    uint8_t messageId = 0;

    uint8_t payloadLength = 0;
    uint8_t payloadIndex = 0;

    uint8_t checksum = 0;

    uint8_t payload[MAX_PAYLOAD_LEN] = {};
};


Parser parser;

DronePet::MtfSample sample;

bool sampleReceived = false;


void resetParser() {

    parser.state =
        ParseState::Head;

    parser.messageId = 0;

    parser.payloadLength = 0;
    parser.payloadIndex = 0;

    parser.checksum = 0;
}


bool decodePacket() {

    if (
        parser.messageId
        != RANGE_SENSOR_MSG_ID
    ) {
        return false;
    }


    if (
        parser.payloadLength
        != sizeof(RangeSensorPayload)
    ) {

        return false;
    }


    RangeSensorPayload payload;

    std::memcpy(
        &payload,
        parser.payload,
        sizeof(payload)
    );


    sample.sensorTimeMs =
        payload.timeMs;

    sample.distanceMm =
        payload.distanceMm;

    sample.tofStatus =
        payload.tofStatus;

    sample.flowQuality =
        payload.flowQuality;

    sample.flowStatus =
        payload.flowStatus;


    if (payload.distanceMm >= 10) {

        const float altitudeM =
            payload.distanceMm
            / 1000.0f;


        // Convert MTF optical flow into DronePet body axes.
        //
        // MTF +X = right   -> DronePet -X
        // MTF +Y = back    -> DronePet +Y
        sample.velocityXMps =
            -payload.flowVelX
            * altitudeM
            / 100.0f;

        sample.velocityYMps =
            payload.flowVelY
            * altitudeM
            / 100.0f;

    } else {

        sample.velocityXMps = 0.0f;
        sample.velocityYMps = 0.0f;
    }


    sample.receivedAtUs =
        micros();

    sampleReceived = true;

    return true;
}


bool parseByte(
    uint8_t data
) {

    switch (parser.state) {

        case ParseState::Head:

            if (data == MICOLINK_HEAD) {

                parser.checksum = data;

                parser.state =
                    ParseState::DeviceId;
            }

            break;


        case ParseState::DeviceId:

            parser.checksum += data;

            parser.state =
                ParseState::SystemId;

            break;


        case ParseState::SystemId:

            parser.checksum += data;

            parser.state =
                ParseState::MessageId;

            break;


        case ParseState::MessageId:

            parser.messageId = data;

            parser.checksum += data;

            parser.state =
                ParseState::Sequence;

            break;


        case ParseState::Sequence:

            parser.checksum += data;

            parser.state =
                ParseState::Length;

            break;


        case ParseState::Length:

            parser.payloadLength = data;
            parser.payloadIndex = 0;

            parser.checksum += data;


            if (
                parser.payloadLength
                > MAX_PAYLOAD_LEN
            ) {

                resetParser();

            } else if (
                parser.payloadLength == 0
            ) {

                parser.state =
                    ParseState::Checksum;

            } else {

                parser.state =
                    ParseState::Payload;
            }

            break;


        case ParseState::Payload:

            parser.payload[
                parser.payloadIndex++
            ] = data;

            parser.checksum += data;


            if (
                parser.payloadIndex
                == parser.payloadLength
            ) {

                parser.state =
                    ParseState::Checksum;
            }

            break;


        case ParseState::Checksum: {

            const bool checksumValid =
                parser.checksum == data;


            if (!checksumValid) {

                resetParser();

                return false;
            }

            const bool decoded =
                decodePacket();


            resetParser();

            return decoded;
        }
    }


    return false;
}

}


bool DronePet::Mtf01::begin() {

    resetParser();

    sample = {};

    sampleReceived = false;


    Serial1.begin(
        MTF_BAUD,
        SERIAL_8N1,
        MTF_RX,
        MTF_TX
    );


    return true;
}


bool DronePet::Mtf01::update() {

    bool newSample = false;


    while (
        Serial1.available() > 0
    ) {

        const uint8_t data =
            static_cast<uint8_t>(
                Serial1.read()
            );

        if (parseByte(data)) {
            newSample = true;
        }
    }


    return newSample;
}


bool DronePet::Mtf01::hasSample() {

    return sampleReceived;
}


bool DronePet::Mtf01::fresh() {

    if (!sampleReceived) {
        return false;
    }


    return (
        micros()
        - sample.receivedAtUs
    ) <= FRESH_TIMEOUT_US;
}


const DronePet::MtfSample&
DronePet::Mtf01::latestSample() {

    return sample;
}