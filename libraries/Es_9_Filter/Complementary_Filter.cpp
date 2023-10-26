#include "Complementary_Filter.h"


Complementary_Filter::Complementary_Filter(IMU& imu, Magnetometer& magnetometer) : _imu{imu}, _magnetometer{magnetometer}
{  
}





void Complementary_Filter::loop()
{
    
    updateRoll();
    updatePitch();
    updateYaw();
    hal.console->printf("roll: %.2f pitch: %.2f yaw: %.2f \n",Complementary_roll,Complementary_pitch,Complementary_yaw);
    
}   

void Complementary_Filter::updateRoll()
{
    
    //add majority vote or redundancy check to determine which sensor to use. 

    //roll is a combination of the accelerometer and gyroscope. 
    //both run at the same frequency so the update can just be run on the new measurements. 
    

    //steps
    //1. get roll from _accelrometers and _IMU 
    //2. fuse both sensor measurements
    //3. save it to the complementary filter. 
    
    filtered_roll = Complementary_Period/(Complementary_Period+2*tau_roll)*(tau_roll*_imu.getMeasurements().at(IMU::Sensors::IMU1).at(IMU::Measurements::gyr_x)+_imu.getMeasurements().at(IMU::Sensors::IMU1).at(IMU::Measurements::acc_roll)); 

    Complementary_roll = filtered_roll  + filtered_roll_prev - (Complementary_Period-2*tau_roll)/(Complementary_Period+2*tau_roll)*Complementary_roll_prev;

    filtered_roll_prev = filtered_roll;

    Complementary_roll_prev = Complementary_roll;

}   


void Complementary_Filter::updatePitch()
{
    //pitch follows the same form as roll 
    filtered_pitch = Complementary_Period/(Complementary_Period+2*tau_pitch)*(tau_pitch*_imu.getMeasurements().at(IMU::Sensors::IMU1).at(IMU::Measurements::gyr_y)+_imu.getMeasurements().at(IMU::Sensors::IMU1).at(IMU::Measurements::acc_pitch)); 

    Complementary_pitch = filtered_pitch  + filtered_pitch_prev - (Complementary_Period-2*tau_pitch)/(Complementary_Period+2*tau_pitch)*Complementary_pitch_prev;

    filtered_pitch_prev = filtered_pitch;

    Complementary_pitch_prev = Complementary_pitch;
}


void Complementary_Filter::updateYaw()
{
    //the magnetometer  is sampled at 10Hz and the gyroscopes are sampled at 400 Hz so the angular velocity is accumulated until measurements from the magnetometer is ready
    
    gyro_angular_velocity_accumulation += _imu.getMeasurements().at(IMU::Sensors::IMU1).at(IMU::Measurements::gyr_z);

    if(sampling_time <= magnetometer_Period)
    {
        sampling_time+= IMU_Period;
    }
    else 
    {
        filtered_yaw = magnetometer_Period/(magnetometer_Period+2*tau_yaw)*(tau_yaw*gyro_angular_velocity_accumulation + _magnetometer.getMeasurements().at(Magnetometer::Sensors::Mag1).at(Magnetometer::Measurements::mag_yaw));

        Complementary_yaw = filtered_yaw  + filtered_yaw_prev - (magnetometer_Period-2*tau_yaw)/(magnetometer_Period+2*tau_yaw)*Complementary_yaw_prev;

        filtered_yaw_prev = filtered_yaw;

        Complementary_yaw_prev = Complementary_yaw;        

        sampling_time =0.0;
        gyro_angular_velocity_accumulation = 0.0;
    }

    
}
