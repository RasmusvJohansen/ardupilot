#pragma once

#include "ES_9_PID.h"
#include "MotorMixing.h"
#include "Es_9_Filter/Complementary_Filter.h"
#include "Es_9_Sensor/Barometer.h"
#include "Es_9_Sensor/Magnetometer.h"
#include "Es_9_Motor/Es_9_Motor.h"
#include "Es_9_Sensor/GPS_fake.h"
#include "Es_9_Sensor/fake_measurement.h"

#include <tuple>

class Controller
{
public:
    Controller(Complementary_Filter& complementary_filter, IMU& imu, Barometer& barometer, Magnetometer& magnetometer, Es_9_Motor& motor_controller, GPS_fake& gps, fake_measurement& fake_measurement, ES_9_PID& pid_roll_angularRate, ES_9_PID& pid_pitch_angularRate, ES_9_PID& pid_yaw_angularRate, ES_9_PID& pid_roll, ES_9_PID& pid_pitch, ES_9_PID& pid_yaw, ES_9_PID& pid_altitude, ES_9_PID& pid_x, ES_9_PID& pid_y);
    void InnerLoop();
    void MiddleLoop();
    void OuterLoop();

private:
    Es_9_Motor& _motorController;
    MotorMixing motor_mixing;

    Complementary_Filter& _complementary_filter;
    IMU& _imu;
    Barometer& _barometer;
    Magnetometer& _magnetometer;
    GPS_fake& _gps;
    fake_measurement& _fake_measurement;

    // Inner PID
    ES_9_PID& _pid_roll_angularRate;
    ES_9_PID& _pid_pitch_angularRate;
    ES_9_PID& _pid_yaw_angularRate;

    // Middle PID
    ES_9_PID& _pid_roll;
    ES_9_PID& _pid_pitch;
    ES_9_PID& _pid_yaw;
    ES_9_PID& _pid_altitude;

    // Outer PID
    ES_9_PID& _pid_x;
    ES_9_PID& _pid_y;

    float input_linearisation_rads { 487.67f };
    float u_roll, u_pitch, u_yaw, u_z { 0.f };
    void adjustOutput();
    bool runController();

    std::tuple<float, float, float> body_angularRate_to_inertial_angular_rate(float body_rate_x, float body_rate_y, float body_rate_z);
};
