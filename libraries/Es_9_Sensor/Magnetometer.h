#pragma once
#include "Sensor.h"
#include "AP_HAL/AP_HAL.h"
#include "AP_Compass/AP_Compass.h"

class Magnetometer : public Sensor
{

public:
    // This can be changed to contain all measurements.
    // Remeber Measurments_Type_List_Stop
    enum class Measurements : int
    {
        mag_x,
        mag_y,
        mag_z,
        Measurements_Type_List_Stop,
    };

    virtual void init() override;
    virtual void updateMeasurements() override;
    std::map<Magnetometer::Measurements, float> getMeasurements();

private:
    Vector3f mag;
    int SensorLocation { 0 };
    const AP_HAL::HAL &hal = AP_HAL::get_HAL();

    std::map<Measurements, float> measurements;
};
