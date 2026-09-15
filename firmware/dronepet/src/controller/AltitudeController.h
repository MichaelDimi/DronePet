#pragma once

#include <Control/Pid.h>
#include <Utils/Filter.h>


namespace DronePet {

class AltitudeController {

public:
    void begin();
    void reset();

    float update(
        float targetAltitudeM,
        float altitudeM
    );


private:
    Espfc::Control::Pid _pid;

    Espfc::Utils::FilterStatePt1 _altitudeFilter;

    bool _filterInitialized = false;
};

}