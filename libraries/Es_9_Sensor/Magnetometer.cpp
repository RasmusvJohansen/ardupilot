#include "Magnetometer.h"

void Magnetometer::init()
{
    // add every type of measurement to the measurements map
    for (int measurement = 0; measurement != static_cast<int>(Measurements::Measurements_Type_List_Stop); measurement++)
    {
        measurements.insert({Magnetometer::Measurements(measurement), 0.0});
    }

    // add every type of measurment to each sensor.
    for (int sensor = 0; sensor != static_cast<int>(Sensors::Sensor_List_stop); sensor++)
    {
        sensors.insert({Magnetometer::Sensors(sensor), measurements});
    }

    // Init sensors here
    // ahrs and compass init in system.cpp line 118
    //magnetometer.set_log_bit(1<<13);
    AP::compass().init();

    // Create offset for yaw, so we always start at 0 yaw
    AP::compass().read();
    mag = AP::compass().get_field(static_cast<int>(Sensors::Mag1));
    yaw_start_value = atan2f(mag.x, mag.y);
    hal.scheduler->delay(100);
}

void Magnetometer::updateMeasurements()
{
    // Read compass values and update mag variables
    AP::compass().read();
    tal = 10.0; 
    // Goes through each sensor, and measurement type and update each values with the current measurement.

    for (int sensor = 0; sensor != static_cast<int>(Sensors::Sensor_List_stop); sensor++)
    {
        // Return the current field as a Vector3f in milligauss
        mag = AP::compass().get_field(sensor);

        for (int measurement = 0; measurement != static_cast<int>(Measurements::Measurements_Type_List_Stop); measurement++)
        {
            // Calculate yaw in rad
            sensors.at(Magnetometer::Sensors(sensor)).at(Magnetometer::Measurements(measurement)) = atan2f(mag.x, mag.y) - yaw_start_value; // here it should get the corresponding measurement for the sensor and measurement type
            
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
    //hal.console->printf("%.2f",tal);
    //hal.console->printf("Yaw: %.2f \n", sensors.at(Magnetometer::Sensors::Mag1).at(Magnetometer::Measurements::mag_yaw));
    // hal.console->printf("Offset: %.2f \n", yaw_start_value);
}