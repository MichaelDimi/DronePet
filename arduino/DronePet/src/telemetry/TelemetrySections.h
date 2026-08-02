#pragma once

#include "Telemetry.h"

namespace TelemetrySections {
  void printImuSample(
      const ImuSample& sample,
      const FlightLoopIteration& iteration
  );

  void printImuHealth(const ImuHealthStatus& health);

  void printTofSample(
      const TofSample& sample,
      const TofHealthStatus& health
  );

  void printTofHealth(const TofHealthStatus& health);
}
