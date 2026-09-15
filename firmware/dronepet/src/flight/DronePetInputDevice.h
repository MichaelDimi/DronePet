#pragma once

#include <Device/InputDevice.h>


namespace DronePet {

class DronePetInputDevice final
    : public Espfc::Device::InputDevice {

public:

  Espfc::InputStatus update() override;

  uint16_t get(
      uint8_t channel
  ) const override;

  void get(
      uint16_t* data,
      size_t len
  ) const override;

  size_t getChannelCount()
      const override;

  bool needAverage()
      const override;


private:

  static constexpr size_t CHANNEL_COUNT = 6;

  uint16_t _channels[
      CHANNEL_COUNT
  ] = {
      1500,
      1500,
      1000,
      1500,
      1000,
      2000
  };

  uint32_t _lastSequence = 0;
};

}