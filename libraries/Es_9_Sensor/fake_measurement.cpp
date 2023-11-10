#include "fake_measurement.h"

std::tuple<float, float, float, float> fake_measurement::getMeasurement() const
{
    return { attitude.at(0), attitude.at(1), attitude.at(2), altitude };
}

std::tuple<float, float, float> fake_measurement::getPosition() const
{
    return { x, y, altitude };
}

std::tuple<float, float, float, float, float, float> fake_measurement::getMeasurementForLogging() const
{
    return { x, y, altitude, attitude.at(0), attitude.at(1), attitude.at(2)};
}

// Called from GCS_MAVLINK_Copter::handleMessage, and is called whenever a fake gps signal is recieved from the GCS
void fake_measurement::setMeasurement(float roll, float pitch, float yaw, float Altitude, float X, float Y)
{
    attitude.at(0) = roll;
    attitude.at(1) = pitch;
    attitude.at(2) = yaw;
    altitude = Altitude;
    x = X;
    y = Y;
}