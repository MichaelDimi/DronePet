#include "AltitudeEstimator.h"

#include <cmath>

namespace DronePet {

float AltitudeEstimator::compensateTilt(
    float rangeM,
    float rollRad,
    float pitchRad
) {
    return rangeM * std::cos(rollRad) * std::cos(pitchRad);
}

const AltitudeEstimate& AltitudeEstimator::update(
    float rangeM,
    float rollRad,
    float pitchRad
) {
    _state.altitudeM = compensateTilt(rangeM, rollRad, pitchRad);
    return _state;
}

const AltitudeEstimate& AltitudeEstimator::state() const {
    return _state;
}

}