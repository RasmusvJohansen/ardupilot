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
void fake_measurement::setMeasurement(float roll_b, float pitch_b, float yaw_b, float Altitude, float X, float Y)
{
    // float inertial_roll = cosf(pitch_b)*cosf(yaw_b) *  roll_b + (cosf(yaw_b)*sinf(pitch_b)*sinf(roll_b)-cosf(roll_b)*sinf(yaw_b)) *      pitch_b  +(sinf(pitch_b)*sinf(roll_b) + cosf(yaw_b)*cosf(roll_b)*sinf(pitch_b))*yaw_b; 
    // float inertial_pitch = cosf(pitch_b)*sinf(yaw_b) * roll_b + (cosf(yaw_b)*cosf(roll_b) + sinf(pitch_b)*sinf(yaw_b)*sinf(roll_b)) *    pitch_b  +(cosf(roll_b)*sinf(pitch_b)*sinf(yaw_b)-cosf(yaw_b)*sinf(roll_b))     *yaw_b;
    // float inertial_yaw = -sinf(pitch_b)              *roll_b + cosf(pitch_b)*sinf(roll_b)                                    *    pitch_b +  cosf(pitch_b)*cosf(roll_b)                                     *yaw_b;

    // float b_roll = cosf(pitch_b)*cosf(yaw_b)                                     * roll_b +  cosf(pitch_b)*sinf(yaw_b)                                   *  pitch_b  -sinf(pitch_b)                  * yaw_b; 
    // float b_pitch = (cosf(yaw_b)*sinf(pitch_b)*sinf(roll_b)-cosf(roll_b)*sinf(yaw_b)) * roll_b + (cosf(yaw_b)*cosf(roll_b) + sinf(pitch_b)*sinf(yaw_b)*sinf(roll_b)) *    pitch_b  +    cosf(pitch_b)*sinf(roll_b) *yaw_b;
    // float b_yaw = (sinf(pitch_b)*sinf(roll_b) + cosf(yaw_b)*cosf(roll_b)*sinf(pitch_b)) *roll_b +(cosf(roll_b)*sinf(pitch_b)*sinf(yaw_b)-cosf(yaw_b)*sinf(roll_b))      *    pitch_b +  cosf(pitch_b)*cosf(roll_b)  *yaw_b;
  


    attitude.at(0) = roll_b;
    attitude.at(1) = pitch_b;
    attitude.at(2) = yaw_b;
    altitude = Altitude;
    x = X;
    y = Y;
}