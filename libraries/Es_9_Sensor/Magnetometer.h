#include "Sensor.h"
#pragma once
#include "AP_HAL/AP_HAL.h"
#include "AP_Compass/AP_Compass.h"

class Magnetometer : public Sensor
{

public:
    // This can be changed to contain all sensors.
    // Remember sensor_List_stop
    enum class Sensors : int
    {
        Mag1,
        Sensor_List_stop,
    };
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
    std::map<Magnetometer::Sensors, std::map<Magnetometer::Measurements, float>> getMeasurements();

    virtual void loop() override;

private:
    //Compass magnetometer;

    Vector3f mag;

    float heading;

    const AP_HAL::HAL &hal = AP_HAL::get_HAL();

    std::map<Sensors, std::map<Measurements, float>> sensors;
    std::map<Measurements, float> measurements;
};
