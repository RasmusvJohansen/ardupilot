#pragma once

#include "Es_9_Sensor/Accelerometers.h" 
#include "Es_9_Sensor/IMU.h"
#include "Es_9_Sensor/Magnetometer.h"
#include "AP_HAL/AP_HAL.h"

class Complementary_Filter
{
public:
    Complementary_Filter(IMU& imu, Magnetometer& magnetometer);
    void loop();
    
    float getRoll() const;
    float getPitch() const;
    float getYaw() const;
        
private:
    Magnetometer& _magnetometer;
  
    IMU& _imu;
    
    void updateRoll();
    void updatePitch();
    void updateYaw();
    
    float tau_roll{.1f}; 
    float tau_pitch{.1f};
    float tau_yaw{.1f};

    float Complementary_roll{0.f};
    float Complementary_pitch{0.f};
    float Complementary_yaw{0.f};

    float Complementary_roll_prev{0.f};
    float Complementary_pitch_prev{0.f};
    float Complementary_yaw_prev{0.f};

    
    float filtered_roll{0.f};
    float filtered_pitch{0.f};
    float filtered_yaw{0.f};
    float filtered_roll_prev{0.f};
    float filtered_pitch_prev{0.f};
    float filtered_yaw_prev{0.f};
    float Complementary_Period{1.f/400.f}; 


    float IMU_Period = {1.f/400.f};
    float magnetometer_Period ={1.f/40.f};
    float gyro_angular_velocity_accumulation{0.f};
    float sampling_time{0.0f};
    float gyro = 0.0f;
    float counter = 0.0f;
    const AP_HAL::HAL &hal = AP_HAL::get_HAL();

    float wrap_around{0.f};
};
