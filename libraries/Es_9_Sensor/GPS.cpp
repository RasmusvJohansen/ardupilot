#include "GPS.h"

void GPS::init()
{
    // add every type of measurement to the measurements map
    for (int measurement = 0; measurement != static_cast<int>(Measurements::Measurements_Type_List_Stop); measurement++)
    {
        measurements.insert({GPS::Measurements(measurement), 0});
    }

    // add every type of measurment to each sensor.
    for (int sensor = 0; sensor != static_cast<int>(Sensors::Sensor_List_stop); sensor++)
    {
        sensors.insert({GPS::Sensors(sensor), measurements});
    }

    // Init sensors here
}

void GPS::updateMeasurements()
{
    // Goes through each senor, and measurement type and update each values with the current measurement.

    for (int sensor = 0; sensor != static_cast<int>(Sensors::Sensor_List_stop); sensor++)
    {
        for (int measurement = 0; measurement != static_cast<int>(Measurements::Measurements_Type_List_Stop); measurement++)
        {
            sensors.at(GPS::Sensors(sensor)).at(GPS::Measurements(measurement)) = 1; // here it should get the corresponding measurement for the sensor and measurement type
        }
    }
}

std::map<GPS::Sensors, std::map<GPS::Measurements, float>> GPS::getMeasurements()
{
    return {sensors};
}

void GPS::loop()
{
    // main loop for the sensors should contain, updateMeasurements and any transformation which should be applied to the measurements.

    updateMeasurements();
}