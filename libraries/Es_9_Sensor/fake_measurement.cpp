#include "fake_measurement.h"

std::tuple<float, float, float, float> fake_measurement::getMeasurement() const
{
    return { attitude.at(0), attitude.at(1), attitude.at(2), altitude };
}

// Called from GCS_MAVLINK_Copter::handleMessage, and is called whenever a fake gps signal is recieved from the GCS
void fake_measurement::setMeasurement(float roll, float pitch, float yaw, float Altitude)
{
    attitude.at(0) = roll;
    attitude.at(1) = pitch;
    attitude.at(2) = yaw;
    altitude = Altitude;
}