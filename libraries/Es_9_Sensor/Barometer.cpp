#include "Barometer.h"

void Barometer::init()
{
    // add every type of measurement to the measurements map
    for (int measurement = 0; measurement != static_cast<int>(Measurements::Measurements_Type_List_Stop); measurement++)
    {
        measurements.insert({Barometer::Measurements(measurement), 0});
    }

    // add every type of measurment to each sensor.
    for (int sensor = 0; sensor != static_cast<int>(Sensors::Sensor_List_stop); sensor++)
    {
        sensors.insert({Barometer::Sensors(sensor), measurements});
    }

    // Init sensors here
    // Init and calibration in system.cpp (search for barometer)
    AP::baro().init();
    AP::baro().calibrate();
    AP::baro().update_calibration();
}

void Barometer::updateMeasurements()
{

    // Call update on all drivers (backend) and push them to frontend
    AP::baro().update();

    // Goes through each sensor, and measurement type and update each values with the current measurement.

    for (int sensor = 0; sensor != static_cast<int>(Sensors::Sensor_List_stop); sensor++)
    {
        for (int measurement = 0; measurement != static_cast<int>(Measurements::Measurements_Type_List_Stop); measurement++)
        {
            // Measure altitude in meters
            sensors.at(Barometer::Sensors(sensor)).at(Barometer::Measurements(measurement)) = AP::baro().get_altitude(sensor); // here it should get the corresponding measurement for the sensor and measurement type
            // hal.console->printf("%f,", AP::baro().get_altitude(sensor));
        }
    }
    // hal.console->printf("\n");
}

std::map<Barometer::Sensors, std::map<Barometer::Measurements, float>> Barometer::getMeasurements()
{
    return {sensors};
}