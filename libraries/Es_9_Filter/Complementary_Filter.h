

#include "Es_9_Sensor/Accelerometers.h" 
#include "Es_9_Sensor/IMU.h"
#include "Es_9_Sensor/Magnetometer.h"
//#include "AP_HAL/AP_HAL.h"

class Complementary_Filter
{
public:
    void loop();
    


    Complementary_Filter(IMU imu, Magnetometer magnetometer);

    
    
      
private:
    IMU _imu;
    Magnetometer _magnetometer;

    void updateRoll();
    void updatePitch();
    void updateYaw();
    
    float tau_roll{1}; 
    float tau_pitch{1};
    float tau_yaw{1};

    float Complementary_roll{0};
    float Complementary_pitch{0};
    float Complementary_yaw{0};

    float Complementary_roll_prev{0};
    float Complementary_pitch_prev{0};
    float Complementary_yaw_prev{0};

    
    float filtered_roll{0};
    float filtered_pitch{0};
    float filtered_yaw{0};
    float filtered_roll_prev{0};
    float filtered_pitch_prev{0};
    float filtered_yaw_prev{0};
    float Complementary_Period{1.0/400.0}; 


    float IMU_Period = {1.0/400.0};
    float magnetometer_Period ={1.0/10.0};
    float gyro_angular_velocity_accumulation{0.f};
    float sampling_time{0.0f};

    //const AP_HAL::HAL &hal = AP_HAL::get_HAL();


};
