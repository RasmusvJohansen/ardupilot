#pragma once'
#include "Sensor.h"
#include "AP_HAL/AP_HAL.h"
#include <AP_GPS/AP_GPS.h>

class GPS : public Sensor
{

public:
    // This can be changed to contain all sensors.
    // Remember sensor_List_stop
    enum class Sensors : int
    {
        Sensor1,
        Sensor2,
        Sensor3,
        Sensor_List_stop,
    };
    // This can be changed to contain all measurements.
    // Remeber Measurments_Type_List_Stop
    enum class Measurements : int
    {
        acc_x,
        acc_y,
        Measurements_Type_List_Stop,
    };

    virtual void init() override;
    virtual void updateMeasurements() override;
    std::map<GPS::Sensors, std::map<GPS::Measurements, float>> getMeasurements();

    virtual void loop() override;

private:
    const AP_HAL::HAL &hal = AP_HAL::get_HAL();

    std::map<Sensors, std::map<Measurements, float>> sensors;
    std::map<Measurements, float> measurements;
};
