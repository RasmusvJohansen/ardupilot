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
    // Accumulates barometer 5 times before
    barometer.accumulate();
    hal.console->printf("Counter: %d \n", counter);
    if (counter++ > 4)
    {
        counter = 0;

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
}

std::map<Barometer::Sensors, std::map<Barometer::Measurements, float>> Barometer::getMeasurements()
{
    return {sensors};
}

void Barometer::loop()
{
    // main loop for the sensors should contain, updateMeasurements and any transformation which should be applied to the measurements.
    //hal.console->printf("Baro test");
    updateMeasurements();
    hal.console->printf("Baro alt: %f \n", sensors.at(Barometer::Sensors::Baro2).at(Barometer::Measurements::altitude));
}