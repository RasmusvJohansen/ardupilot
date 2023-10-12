#include "IMU.h"

void IMU::init()
{
    // add every type of measurement to the measurements map
    for (int measurement = 0; measurement != static_cast<int>(Measurements::Measurements_Type_List_Stop); measurement++)
    {
        measurements.insert({IMU::Measurements(measurement), 0});
    }

    // add every type of measurment to each sensor.
    for (int sensor = 0; sensor != static_cast<int>(Sensors::Sensor_List_stop); sensor++)
    {
        sensors.insert({IMU::Sensors(sensor), measurements});
    }

    // Init sensors here
}

void IMU::updateMeasurements()
{
    // Goes through each senor, and measurement type and update each values with the current measurement.

    for (int sensor = 0; sensor != static_cast<int>(Sensors::Sensor_List_stop); sensor++)
    {
        for (int measurement = 0; measurement != static_cast<int>(Measurements::Measurements_Type_List_Stop); measurement++)
        {
            sensors.at(IMU::Sensors(sensor)).at(IMU::Measurements(measurement)) = 1; // here it should get the corresponding measurement for the sensor and measurement type
        }
    }
}

std::map<IMU::Sensors, std::map<IMU::Measurements, float>> IMU::getMeasurements()
{
    return {sensors};
}

void IMU::loop()
{
    // main loop for the sensors should contain, updateMeasurements and any transformation which should be applied to the measurements.

    //hal.console->printf("%f", sensors.at(IMU::Sensors::Sensor1).at(IMU::Measurements::acc_x));
    //updateMeasurements();
    //hal.console->printf("%f", sensors.at(IMU::Sensors::Sensor1).at(IMU::Measurements::acc_x));
}