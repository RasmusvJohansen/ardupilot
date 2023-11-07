#pragma once

#include "ES_9_PID.h"
#include "MotorMixing.h"
#include "Es_9_Filter/Complementary_Filter.h"
#include "Es_9_Sensor/Barometer.h"
#include "Es_9_Motor/Es_9_Motor.h"
#include "Es_9_Sensor/GPS_fake.h"
#include "Es_9_Sensor/fake_measurement.h"

#include <tuple>

class Controller
{
public:
    Controller(Complementary_Filter& complementary_filter, Barometer& barometer, ES_9_PID& pid_roll, ES_9_PID& pid_pitch, ES_9_PID& pid_yaw, ES_9_PID& pid_altitude, Es_9_Motor& motor_controller, GPS_fake& gps, fake_measurement& fake_measurement);
    void loop();

private:
    Complementary_Filter& _complementary_filter;
    Barometer& _barometer;
    
    ES_9_PID& _pid_roll;
    ES_9_PID& _pid_pitch;
    ES_9_PID& _pid_yaw;
    ES_9_PID& _pid_altitude;
    MotorMixing motor_mixing;
    Es_9_Motor& _motorController;
    GPS_fake& _gps;
    fake_measurement& _fake_measurement;

    float input_linearisation_rads { 337.67f };
};
