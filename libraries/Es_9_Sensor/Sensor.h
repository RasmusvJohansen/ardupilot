#pragma once

#include <map>

// This is the main class all sensor should inherit from. For an example of how to structure each sensor, look at
// accelerometers.cpp and .h

class Sensor
{
public:
    virtual void init();
    virtual void updateMeasurements();
    // this is used in the scheduler to act as a main loop, which describes what should be run when the scheduler activates.
    virtual void loop();

private:
};
