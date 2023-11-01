#include "GPS_fake.h"

std::tuple<float, float, float> GPS_fake::getPosition() const
{
    return { position.at(0), position.at(1), position.at(2) };
}

// Called from GCS_MAVLINK_Copter::handleMessage, and is called whenever a fake gps signal is recieved from the GCS
void GPS_fake::setPosition(float x, float y, float z)
{
    position.at(0) = x;
    position.at(1) = y;
    position.at(2) = z;
}