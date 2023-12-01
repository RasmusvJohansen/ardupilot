#include "Complementary_Filter.h"


Complementary_Filter::Complementary_Filter(IMU& imu, Magnetometer& magnetometer) : _imu{imu}, _magnetometer{magnetometer}
{  
}

void Complementary_Filter::loop()
{
    updateRoll();
    updatePitch();
    updateYaw();
}

void Complementary_Filter::updateRoll()
{    
    // sensors.at(IMU::Sensors(sensor)).at(IMU::Measurements::acc_roll) = atan2f(-acc.y, -acc.z);
        // sensors.at(IMU::Sensors(sensor)).at(IMU::Measurements::acc_pitch) = atan2f(-acc.x, -acc.z);

    // filtered_roll = Complementary_Period/(Complementary_Period+2*tau_roll)*(tau_roll*_imu.getMeasurements().at(IMU::Sensors::IMU1).at(IMU::Measurements::gyr_x)+_imu.getMeasurements().at(IMU::Sensors::IMU1).at(IMU::Measurements::acc_roll)); 

    // Complementary_roll = filtered_roll  + filtered_roll_prev - (Complementary_Period-2*tau_roll)/(Complementary_Period+2*tau_roll)*Complementary_roll_prev;

    // filtered_roll_prev = filtered_roll;

    // Complementary_roll_prev = Complementary_roll;

}   


void Complementary_Filter::updatePitch()
{
    //pitch follows the same form as roll 
    // filtered_pitch = Complementary_Period/(Complementary_Period+2*tau_pitch)*(tau_pitch*_imu.getMeasurements().at(IMU::Sensors::IMU1).at(IMU::Measurements::gyr_y)+_imu.getMeasurements().at(IMU::Sensors::IMU1).at(IMU::Measurements::acc_pitch)); 

    // Complementary_pitch = filtered_pitch  + filtered_pitch_prev - (Complementary_Period-2*tau_pitch)/(Complementary_Period+2*tau_pitch)*Complementary_pitch_prev;

    // filtered_pitch_prev = filtered_pitch;

    // Complementary_pitch_prev = Complementary_pitch;
}


void Complementary_Filter::updateYaw()
{
    //the magnetometer  is sampled at 10Hz and the gyroscopes are sampled at 400 Hz so the angular velocity is accumulated until measurements from the magnetometer is ready
    //hal.console->printf("Sampling time: %.4f Yaw: %.2f \n",sampling_time,gyro_angular_velocity_accumulation);
// sensors.at(Magnetometer::Sensors(sensor)).at(Magnetometer::Measurements(measurement)) = atan2f(mag.x, mag.y) - yaw_start_value; // here it should get the corresponding measurement for the sensor and measurement type
    // gyro_angular_velocity_accumulation += _imu.getMeasurements().at(IMU::Sensors::IMU1).at(IMU::Measurements::gyr_z);

    // if(sampling_time < magnetometer_Period)
    // {
    //     sampling_time+= IMU_Period;
    //     counter++;
    // }

    // else 
    // {
    //     // Counter to add or subtract 2pi from magnetometer angle when it wraps around
    //     if(_magnetometer.getMeasurements().at(Magnetometer::Sensors::Mag1).at(Magnetometer::Measurements::mag_yaw)+ 2*M_PI*wrap_around - Complementary_yaw_prev < - M_PI)
    //     {
    //         wrap_around++;
    //     }
    //     else if(_magnetometer.getMeasurements().at(Magnetometer::Sensors::Mag1).at(Magnetometer::Measurements::mag_yaw)+ 2*M_PI*wrap_around - Complementary_yaw_prev > M_PI)
    //     {
    //         wrap_around--;
    //     }

    //     gyro = gyro_angular_velocity_accumulation/counter;
    //     filtered_yaw = magnetometer_Period/(magnetometer_Period+2*tau_yaw)*(tau_yaw*gyro + _magnetometer.getMeasurements().at(Magnetometer::Sensors::Mag1).at(Magnetometer::Measurements::mag_yaw) + 2*M_PI*wrap_around);

    //     Complementary_yaw = filtered_yaw  + filtered_yaw_prev - (magnetometer_Period-2*tau_yaw)/(magnetometer_Period+2*tau_yaw)*Complementary_yaw_prev;

    //     filtered_yaw_prev = filtered_yaw;

    //     Complementary_yaw_prev = Complementary_yaw;        

    //     sampling_time =0.0;
    //     gyro_angular_velocity_accumulation = 0.0;
    //     counter = 0.0;
    // }    
}


float Complementary_Filter::getRoll() const
{
    return Complementary_roll;
}

float Complementary_Filter::getPitch() const
{
    return Complementary_pitch;
}

float Complementary_Filter::getYaw() const
{
    return Complementary_yaw;
}