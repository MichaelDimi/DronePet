#include "Tof.h"
#include "TofStartup.h"

bool TofStartup::initialize() {
  return Tof::begin();
}