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
    hal.scheduler->delay(2000);
    AP::baro().calibrate();

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
    updateMeasurements();
    float baro_pressure1 = AP::baro().get_pressure(0);
    float baro_pressure2 = AP::baro().get_pressure(1);
    float baro_altitude1 = AP::baro().get_altitude(0);
    float baro_altitude2 = AP::baro().get_altitude(1);

    hal.console->printf("Baro ground pressure: %.2f \n", AP::baro().get_ground_pressure(0));
    hal.console->printf("Baro normal pressure: %.2f \n", baro_pressure1);
    hal.console->printf("Baro altitude this: %.2f \n", baro_altitude1);
    hal.console->printf("Baro altitude diff: %.2f \n", AP::baro().get_altitude_difference(AP::baro().get_ground_pressure(0),baro_pressure1));

    hal.console->printf("Baro ground pressure 2: %.2f \n", AP::baro().get_ground_pressure(1));
    hal.console->printf("Baro normal pressure 2: %.2f \n", baro_pressure2);
    hal.console->printf("Baro altitude this: %.2f \n", baro_altitude2);
    hal.console->printf("Baro altitude diff: %.2f \n", AP::baro().get_altitude_difference(AP::baro().get_ground_pressure(1),baro_pressure2));
}