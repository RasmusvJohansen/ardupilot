#include "Accelerometers.h"

void Accelerometers::init()
{
    // add every type of measurement to the measurements map
    for (int measurement = 0; measurement != static_cast<int>(Measurements::Measurements_Type_List_Stop); measurement++)
    {
        measurements.insert({Accelerometers::Measurements(measurement), 0});
    }

    // add every type of measurment to each sensor.
    for (int sensor = 0; sensor != static_cast<int>(Sensors::Sensor_List_stop); sensor++)
    {
        sensors.insert({Accelerometers::Sensors(sensor), measurements});
    }

    // Init sensors here
}

void Accelerometers::updateMeasurements()
{
    // Goes through each senor, and measurement type and update each values with the current measurement.

    for (int sensor = 0; sensor != static_cast<int>(Sensors::Sensor_List_stop); sensor++)
    {
        for (int measurement = 0; measurement != static_cast<int>(Measurements::Measurements_Type_List_Stop); measurement++)
        {
            sensors.at(Accelerometers::Sensors(sensor)).at(Accelerometers::Measurements(measurement)) = 1; // here it should get the corresponding measurement for the sensor and measurement type
        }
    }
}

std::map<Accelerometers::Sensors, std::map<Accelerometers::Measurements, float>> Accelerometers::getMeasurements()
{
    return {sensors};
}

void Accelerometers::loop()
{
    // main loop for the sensors should contain, updateMeasurements and any transformation which should be applied to the measurements.

    //hal.console->printf("%f", sensors.at(Accelerometers::Sensors::Sensor1).at(Accelerometers::Measurements::acc_x));
    //updateMeasurements();
    //hal.console->printf("%f", sensors.at(Accelerometers::Sensors::Sensor1).at(Accelerometers::Measurements::acc_x));
}