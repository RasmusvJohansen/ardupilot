#include "Magnetometer.h"

void Magnetometer::init()
{
    // add every type of measurement to the measurements map
    for (int measurement = 0; measurement != static_cast<int>(Measurements::Measurements_Type_List_Stop); measurement++)
    {
        measurements.insert({Magnetometer::Measurements(measurement), 0.0});
    }

    // Init sensors here
    // ahrs and compass init in system.cpp line 118
    AP::compass().init();
}

void Magnetometer::updateMeasurements()
{
    // Read compass values and update mag variables
    AP::compass().read();
    // Goes through each sensor, and measurement type and update each values with the current measurement.

    // Return the current field as a Vector3f in milligauss
    mag = AP::compass().get_field(SensorLocation);
    measurements.at(Measurements::mag_x) = mag.x;
    measurements.at(Measurements::mag_y) = -mag.y;
    measurements.at(Measurements::mag_z) = -mag.z;
}

std::map<Magnetometer::Measurements, float> Magnetometer::getMeasurements()
{
    return {measurements};
}