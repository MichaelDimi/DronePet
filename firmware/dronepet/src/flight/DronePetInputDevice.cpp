#include "DronePetInputDevice.h"

#include <algorithm>
#include <cmath>

#include "FlightCommandMailbox.h"


namespace {

constexpr uint32_t COMMAND_TIMEOUT_US =
    200000;


uint16_t axisToPwm(
    float value
) {

  value = std::clamp(
      value,
      -1.0f,
      1.0f
  );

  return static_cast<uint16_t>(
      std::lround(
          1500.0f
          + value * 500.0f
      )
  );
}


uint16_t throttleToPwm(
    float value
) {

  value = std::clamp(
      value,
      0.0f,
      1.0f
  );

  return static_cast<uint16_t>(
      std::lround(
          1000.0f
          + value * 1000.0f
      )
  );
}

}


Espfc::InputStatus
DronePet::DronePetInputDevice::update() {

  const auto snapshot =
      FlightCommandMailbox::read();


  if (
      !snapshot.valid
      || snapshot.ageUs
          > COMMAND_TIMEOUT_US
  ) {

    _channels[0] = 1500;
    _channels[1] = 1500;
    _channels[2] = 1000;
    _channels[3] = 1500;
    _channels[4] = 1000;
    _channels[5] = 2000;

    return Espfc::INPUT_FAILSAFE;
  }


  if (
      snapshot.sequence
      == _lastSequence
  ) {

    return Espfc::INPUT_IDLE;
  }


  _lastSequence =
      snapshot.sequence;


  const PilotCommand& command =
      snapshot.command;


  // ESP-FC's default raw receiver order is AETR:
  //
  // 0 = roll
  // 1 = pitch
  // 2 = throttle
  // 3 = yaw
  // 4 = AUX1
  // 5 = AUX2

  _channels[0] =
      axisToPwm(command.roll);

  _channels[1] =
      axisToPwm(command.pitch);

  _channels[2] =
      throttleToPwm(
          command.throttle
      );

  _channels[3] =
      axisToPwm(command.yaw);

  _channels[4] =
      command.armed
          ? 2000
          : 1000;

  _channels[5] =
      command.angleMode
          ? 2000
          : 1000;


  return Espfc::INPUT_RECEIVED;
}


uint16_t
DronePet::DronePetInputDevice::get(
    uint8_t channel
) const {

  if (
      channel >= CHANNEL_COUNT
  ) {
    return 1500;
  }

  return _channels[channel];
}


void
DronePet::DronePetInputDevice::get(
    uint16_t* data,
    size_t len
) const {

  const size_t count =
      std::min(
          len,
          CHANNEL_COUNT
      );

  for (
      size_t i = 0;
      i < count;
      ++i
  ) {

    data[i] =
        _channels[i];
  }
}


size_t
DronePet::DronePetInputDevice::
getChannelCount() const {

  return CHANNEL_COUNT;
}


bool
DronePet::DronePetInputDevice::
needAverage() const {

  return false;
}