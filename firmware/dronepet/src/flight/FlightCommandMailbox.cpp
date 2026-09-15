#include "FlightCommandMailbox.h"

namespace {

portMUX_TYPE mailboxMux = portMUX_INITIALIZER_UNLOCKED;

DronePet::PilotCommand latestCommand;

uint32_t latestSequence = 0;
uint32_t latestPublishedAtUs = 0;

bool hasCommand = false;

}

void DronePet::FlightCommandMailbox::publish(
    const PilotCommand& command
) {

  const uint32_t nowUs =
      micros();

  portENTER_CRITICAL(
      &mailboxMux
  );

  latestCommand = command;
  latestSequence++;
  latestPublishedAtUs = nowUs;
  hasCommand = true;

  portEXIT_CRITICAL(
      &mailboxMux
  );
}


DronePet::FlightCommandMailbox::Snapshot
DronePet::FlightCommandMailbox::read() {

  Snapshot snapshot;

  uint32_t publishedAtUs = 0;

  portENTER_CRITICAL(
      &mailboxMux
  );

  snapshot.command = latestCommand;

  snapshot.sequence = latestSequence;

  snapshot.valid = hasCommand;

  publishedAtUs = latestPublishedAtUs;

  portEXIT_CRITICAL(
      &mailboxMux
  );


  if (snapshot.valid) {
    snapshot.ageUs =
        micros() - publishedAtUs;
  }

  return snapshot;
}
