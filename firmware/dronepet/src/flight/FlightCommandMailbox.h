#pragma once

#include <Arduino.h>

#include "PilotCommand.h"


namespace DronePet::FlightCommandMailbox {

struct Snapshot {
  PilotCommand command;
  uint32_t sequence = 0;
  uint32_t ageUs = 0;
  bool valid = false;
};


void publish(
    const PilotCommand& command
);

Snapshot read();

}