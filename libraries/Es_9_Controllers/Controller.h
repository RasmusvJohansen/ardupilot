#pragma once

#include "ES_9_PID.h"
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
    struct PID_Container
    {
        struct  
        {
            //PID attitude
            ES_9_PID roll;
            ES_9_PID pitch;
            ES_9_PID yaw;
        }PID_Attitude;

        struct 
        {
            //PID angular rate 
            ES_9_PID roll; 
            ES_9_PID pitch; 
            ES_9_PID yaw; 
        } PID_AngularRate;

        struct
        {
            //PID velocity
            ES_9_PID x;
            ES_9_PID y;
            ES_9_PID z;
        }PID_Velocity;

        struct
        {
            //PID Position
            ES_9_PID z;
            ES_9_PID x; 
            ES_9_PID y; 
        }PID_Position;
    
    };  


    
    Controller(Complementary_Filter& complementary_filter, IMU& imu, Barometer& barometer, Magnetometer& magnetometer, Es_9_Motor& motor_controller, GPS_fake& gps, fake_measurement& fake_measurement, PID_Container& pid);
    void InnerLoop();
    void MiddleLoop();
    void CascadeLoop3();
    void OuterLoop();
    


private:
    Es_9_Motor& _motorController;
    
    
    PID_Container _pid;
    
    Complementary_Filter& _complementary_filter;
    IMU& _imu;
    Barometer& _barometer;
    Magnetometer& _magnetometer;
    GPS_fake& _gps;
    fake_measurement& _fake_measurement;

    // Inner PID

    // ES_9_PID& _pid_roll_angularRate;
    // ES_9_PID& _pid_pitch_angularRate;
    // ES_9_PID& _pid_yaw_angularRate;

    // // Middle PID
    // ES_9_PID& _pid_roll;
    // ES_9_PID& _pid_pitch;
    // ES_9_PID& _pid_yaw;
    // ES_9_PID& _pid_altitude;

    // //CascadeLoop3 PID
    // ES_9_PID& _pid_velocity_x;
    // ES_9_PID& _pid_velocity_y;
    // ES_9_PID& _PID_velocity_z; 
    
    // // Outer PID
    // ES_9_PID& _pid_x;
    // ES_9_PID& _pid_y;

    float input_linearisation_rads { 480.67f };
    // float input_linearisation_rads { 387.67f };
    float u_roll, u_pitch, u_yaw, u_z { 0.f };
    void adjustOutput();
    bool runController();

    std::tuple<float, float, float, float> MotorMix(float u_roll, float u_pitch, float u_yaw, float u_z);
    std::tuple<float, float, float> RotationBI(float roll_b, float pitch_b, float yaw_b);
    std::tuple<float, float, float> RotationIB(float roll_i, float pitch_i, float yaw_i);
};
