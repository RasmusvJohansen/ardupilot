#include "Magnetometer.h"

void Magnetometer::init()
{
    // add every type of measurement to the measurements map
    for (int measurement = 0; measurement != static_cast<int>(Measurements::Measurements_Type_List_Stop); measurement++)
    {
        measurements.insert({Magnetometer::Measurements(measurement), 0});
    }

    // add every type of measurment to each sensor.
    for (int sensor = 0; sensor != static_cast<int>(Sensors::Sensor_List_stop); sensor++)
    {
        sensors.insert({Magnetometer::Sensors(sensor), measurements});
    }

    // Init sensors here
    // ahrs and compass init in system.cpp
}

void Magnetometer::updateMeasurements()
{
    // Read compass values and update mag variables
    magnetometer.read();

    // Goes through each senor, and measurement type and update each values with the current measurement.

    for (int sensor = 0; sensor != static_cast<int>(Sensors::Sensor_List_stop); sensor++)
    {
        // Return the current field as a Vector3f in milligauss
        mag = magnetometer.get_field(sensor);

        for (int measurement = 0; measurement != static_cast<int>(Measurements::Measurements_Type_List_Stop); measurement++)
        {
            sensors.at(Magnetometer::Sensors(sensor)).at(Magnetometer::Measurements(measurement)) = mag[measurement]; // here it should get the corresponding measurement for the sensor and measurement type
        }
    }
}

std::map<Magnetometer::Sensors, std::map<Magnetometer::Measurements, float>> Magnetometer::getMeasurements()
{
    return {sensors};
}

void Magnetometer::loop()
{
    // main loop for the sensors should contain, updateMeasurements and any transformation which should be applied to the measurements.

    updateMeasurements();
    hal.console->printf("Mag x: %f \n", sensors.at(Magnetometer::Sensors::Mag1).at(Magnetometer::Measurements::mag_x));
}