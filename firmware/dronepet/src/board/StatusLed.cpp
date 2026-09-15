#include "StatusLed.h"

#include "BoardPins.h"

#include <Arduino.h>
#include <algorithm>
#include <driver/i2s.h>

namespace {

constexpr size_t PIXEL_SIZE = 12;
constexpr size_t RESET_SIZE = 32;
constexpr size_t BUFFER_SIZE = PIXEL_SIZE + RESET_SIZE;

constexpr uint32_t SAMPLE_RATE = 93750;
constexpr i2s_port_t I2S_PORT = I2S_NUM_0;

constexpr uint8_t BRIGHTNESS = 32;

uint8_t buffer[BUFFER_SIZE] = {};

const uint16_t bitPatterns[4] = {
    0x88,
    0x8e,
    0xe8,
    0xee
};


void writeChannel(uint8_t*& output, uint8_t value) {
    *output++ = bitPatterns[(value >> 6) & 0x03];
    *output++ = bitPatterns[(value >> 4) & 0x03];
    *output++ = bitPatterns[(value >> 2) & 0x03];
    *output++ = bitPatterns[value & 0x03];
}


void writeColor(uint8_t red, uint8_t green, uint8_t blue) {
    uint8_t* output = buffer;

    // ESP32-S3-Zero onboard LED uses RGB ordering.
    writeChannel(output, red);
    writeChannel(output, green);
    writeChannel(output, blue);

    size_t bytesWritten = 0;

    i2s_zero_dma_buffer(I2S_PORT);
    i2s_write(I2S_PORT, buffer, BUFFER_SIZE, &bytesWritten, portMAX_DELAY);
}

}


void DronePet::StatusLed::begin() {
    i2s_config_t config = {
        .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .intr_alloc_flags = 0,
        .dma_buf_count = 2,
        .dma_buf_len = BUFFER_SIZE / 2,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0,
        .mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT,
        .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT,
    };

    i2s_pin_config_t pins = {
        .bck_io_num = -1,
        .ws_io_num = -1,
        .data_out_num = STATUS_LED,
        .data_in_num = -1
    };

    i2s_driver_install(I2S_PORT, &config, 0, nullptr);
    i2s_set_pin(I2S_PORT, &pins);

    std::fill_n(buffer, BUFFER_SIZE, 0);

    off();
}


void DronePet::StatusLed::off() {
    writeColor(0, 0, 0);
}


void DronePet::StatusLed::red() {
    writeColor(BRIGHTNESS, 0, 0);
}


void DronePet::StatusLed::green() {
    writeColor(0, BRIGHTNESS, 0);
}


void DronePet::StatusLed::blue() {
    writeColor(0, 0, BRIGHTNESS);
}