#pragma once

#include "Tof.h"

struct TofHealthStatus {
  bool initialized = false;
  bool sampleReceived = false;
  bool sampleFresh = false;
  bool rangeValid = false;
  bool distanceAboveMinimum = false;
  bool communicationError = false;

  uint32_t sampleAgeMs = 0;

  bool responsive() const {
    return initialized
        && sampleReceived
        && sampleFresh
        && !communicationError;
  }

  bool healthy() const {
    return responsive()
        && rangeValid
        && distanceAboveMinimum;
  }
};

namespace TofHealth {
  TofHealthStatus evaluate(
      bool initialized,
      bool hasSample,
      bool communicationError,
      const TofSample& sample
  );
}