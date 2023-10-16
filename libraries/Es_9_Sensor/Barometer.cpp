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
}

void Barometer::updateMeasurements()
{
    // Accumulate not neccesary, as the MS5611 has a timer

    // Call update on all drivers (backend) and push them to frontend
    barometer.update();

    // Goes through each senor, and measurement type and update each values with the current measurement.

    for (int sensor = 0; sensor != static_cast<int>(Sensors::Sensor_List_stop); sensor++)
    {
        for (int measurement = 0; measurement != static_cast<int>(Measurements::Measurements_Type_List_Stop); measurement++)
        {
            sensors.at(Barometer::Sensors(sensor)).at(Barometer::Measurements(measurement)) = barometer.get_altitude(sensor); // here it should get the corresponding measurement for the sensor and measurement type
        }
    }
}

std::map<Barometer::Sensors, std::map<Barometer::Measurements, float>> Barometer::getMeasurements()
{
    return {sensors};
}

void Barometer::loop()
{
    // main loop for the sensors should contain, updateMeasurements and any transformation which should be applied to the measurements.

    //hal.console->printf("%f", sensors.at(Barometer::Sensors::Sensor1).at(Barometer::Measurements::acc_x));
    //updateMeasurements();
    //hal.console->printf("%f", sensors.at(Barometer::Sensors::Sensor1).at(Barometer::Measurements::acc_x));
}