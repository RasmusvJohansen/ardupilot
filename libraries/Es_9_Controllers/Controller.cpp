#include "Controller.h"

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL &hal;

    Controller::Controller(Complementary_Filter& complementary_filter, IMU& imu, Barometer& barometer, Magnetometer& magnetometer, Es_9_Motor& motor_controller, GPS_fake& gps, fake_measurement& fake_measurement, ES_9_PID& pid_roll_angularRate, ES_9_PID& pid_pitch_angularRate, ES_9_PID& pid_yaw_angularRate, ES_9_PID& pid_roll, ES_9_PID& pid_pitch, ES_9_PID& pid_yaw, ES_9_PID& pid_velocity_x, ES_9_PID& pid_velocity_y, ES_9_PID& pid_velocity_z, ES_9_PID& pid_altitude, ES_9_PID& pid_x, ES_9_PID& pid_y) :
    _complementary_filter{ complementary_filter }, _imu{ imu }, _barometer{ barometer }, _magnetometer{ magnetometer }, _motorController{ motor_controller }, _gps{ gps }, _fake_measurement { fake_measurement }, _pid_roll_angularRate{ pid_roll_angularRate }, _pid_pitch_angularRate{ pid_pitch_angularRate }, _pid_yaw_angularRate{ pid_yaw_angularRate }, _pid_roll{ pid_roll }, _pid_pitch{ pid_pitch }, _pid_yaw{ pid_yaw }, _pid_velocity_x{pid_velocity_x},_pid_velocity_y{pid_velocity_y},_pid_velocity_z{pid_velocity_z}, _pid_altitude{ pid_altitude }, _pid_x{ pid_x }, _pid_y{ pid_y }
{

}

void Controller::InnerLoop()
{
    if(!runController())
    {
        return;
    }
    float interial_rate_roll, interial_rate_pitch, interial_rate_yaw { 0.f };
    std::tie(interial_rate_roll, interial_rate_pitch, interial_rate_yaw) = RotationBI(_imu.getMeasurements().at(IMU::Sensors::IMU1).at(IMU::Measurements::gyr_x),_imu.getMeasurements().at(IMU::Sensors::IMU1).at(IMU::Measurements::gyr_y),_imu.getMeasurements().at(IMU::Sensors::IMU1).at(IMU::Measurements::gyr_z));

    u_roll = _pid_roll_angularRate.calculatePIDOutput(interial_rate_roll);
    u_pitch = _pid_pitch_angularRate.calculatePIDOutput(interial_rate_pitch);
    u_yaw = _pid_yaw_angularRate.calculatePIDOutput(interial_rate_yaw);
   
    adjustOutput();
}
void Controller::MiddleLoop()
{
    if(!runController())
    {
        return;
    }
    float roll, pitch, yaw, z {0.f};
    std::tie(roll, pitch, yaw, z) = _fake_measurement.getMeasurement();

     float v_x, v_y, v_z {0.f};
    std::tie(v_x, v_y, v_z) = _fake_measurement.getVelocity();


    float reference_angularRate_roll = _pid_roll.calculatePIDOutput(roll);
    float reference_angularRate_pitch = _pid_pitch.calculatePIDOutput(pitch);
    float reference_angularRate_yaw = _pid_yaw.calculatePIDOutput(yaw);

    _pid_roll_angularRate.setReference(reference_angularRate_roll);
    _pid_pitch_angularRate.setReference(reference_angularRate_pitch);
    _pid_yaw_angularRate.setReference(reference_angularRate_yaw);
    u_z = _pid_velocity_z.calculatePIDOutput(v_z);
    adjustOutput();
}

void Controller::CascadeLoop3() 
{
    if(!runController())
    {
        return;
    }

    float roll, pitch, yaw, z {0.f};
    std::tie(roll, pitch, yaw, z) = _fake_measurement.getMeasurement();


    float v_x, v_y, v_z {0.f};
    std::tie(v_x, v_y, v_z) = _fake_measurement.getVelocity();

    float reference_pitch = _pid_velocity_x.calculatePIDOutput(v_x);
    float reference_roll = -_pid_velocity_y.calculatePIDOutput(v_y); //obs -
    _pid_roll.setReference(reference_roll);
    _pid_pitch.setReference(reference_pitch);
    float reference_vz = _pid_altitude.calculatePIDOutput(z);
    _pid_velocity_z.setReference(reference_vz);
    
    
}


void Controller::OuterLoop()
{
    if(!runController())
    {
        return;
    }

    float x, y, z {0.f};
    std::tie(x, y, z) = _fake_measurement.getPosition();

    float reference_vy = _pid_y.calculatePIDOutput(y); 
    float reference_vx = _pid_x.calculatePIDOutput(x);
    _pid_velocity_x.setReference(reference_vx);
    _pid_velocity_y.setReference(reference_vy);
}

void Controller::adjustOutput()
{
    float omega_m1, omega_m2, omega_m3, omega_m4, u_roll_b, u_pitch_b, u_yaw_b { 0.f };

    std::tie(u_roll_b,u_pitch_b,u_yaw_b) = RotationIB(u_roll,u_pitch,u_yaw);
    std::tie(omega_m1, omega_m2, omega_m3, omega_m4) = MotorMix(u_roll_b, u_pitch_b, u_yaw_b, u_z); //rasmus har lavet ballade her

    hal.console->printf("roll: %.02f|pitch: %0.2f|yaw: %0.2f|u_z: %0.2f|\n",u_roll,u_pitch,u_yaw,u_z);

    float roll, pitch, yaw, z {0.f};
    std::tie(roll, pitch, yaw, z) = _fake_measurement.getMeasurement();
    _motorController.setAllMotorAngularVelocity(omega_m1 + input_linearisation_rads, omega_m2 + input_linearisation_rads, omega_m3 + input_linearisation_rads, omega_m4 + input_linearisation_rads);
}

bool Controller::runController()
{
    if(!_motorController.getIsFlying())
    {
        if(_motorController.getIsArmed())
        {
            _motorController.setAllMotorPeriod(1100,1100,1100,1100);
        }
        return false;
    }
    return true;
}

std::tuple<float, float, float, float> Controller::MotorMix(float Uroll, float Upitch, float Uyaw, float Uz)
{
    float omega_m1 = 1.f/(4.f) * Uz - 1.f/(4.f) * Uroll - 1.f/(4.f) * Upitch - 1.f/(4.f) * Uyaw;
    float omega_m2 = 1.f/(4.f) * Uz + 1.f/(4.f) * Uroll + 1.f/(4.f) * Upitch - 1.f/(4.f) * Uyaw;
    float omega_m3 = 1.f/(4.f) * Uz + 1.f/(4.f) * Uroll - 1.f/(4.f) * Upitch + 1.f/(4.f) * Uyaw;
    float omega_m4 = 1.f/(4.f) * Uz - 1.f/(4.f) * Uroll + 1.f/(4.f) * Upitch + 1.f/(4.f) * Uyaw;

    return {omega_m1, omega_m2, omega_m3, omega_m4};
}

std::tuple<float, float, float> Controller::RotationBI(float roll_b, float pitch_b, float yaw_b)
{   
    float roll, pitch, yaw, z {0.f};
    std::tie(roll, pitch, yaw, z) = _fake_measurement.getMeasurement();

    float inertial_roll = cosf(pitch)*cosf(yaw) *  roll_b + (cosf(yaw)*sinf(pitch)*sinf(roll)-cosf(roll)*sinf(yaw)) *      pitch_b  +(sinf(pitch)*sinf(roll) + cosf(yaw)*cosf(roll)*sinf(pitch))*yaw_b; 
    float inertial_pitch = cosf(pitch)*sinf(yaw) * roll_b + (cosf(yaw)*cosf(roll) + sinf(pitch)*sinf(yaw)*sinf(roll)) *    pitch_b  +(cosf(roll)*sinf(pitch)*sinf(yaw)-cosf(yaw)*sinf(roll))     *yaw_b;
    float inertial_yaw = -sinf(pitch)              *roll_b + cosf(pitch)*sinf(roll)                                    *    pitch_b +  cosf(pitch)*cosf(roll)                                     *yaw_b;
    return {inertial_roll, inertial_pitch, inertial_yaw};
}

std::tuple<float, float, float> Controller::RotationIB(float roll_i, float pitch_i, float yaw_i)
{   
    float roll, pitch, yaw, z {0.f};
    std::tie(roll, pitch, yaw, z) = _fake_measurement.getMeasurement();

    float b_roll = cosf(pitch)*cosf(yaw)                                     * roll_i +  cosf(pitch)*sinf(yaw)                                   *  pitch_i  -sinf(pitch)                  * yaw_i; 
    float b_pitch = (cosf(yaw)*sinf(pitch)*sinf(roll)-cosf(roll)*sinf(yaw)) * roll_i + (cosf(yaw)*cosf(roll) + sinf(pitch)*sinf(yaw)*sinf(roll)) *    pitch_i  +    cosf(pitch)*sinf(roll) *yaw_i;
    float b_yaw = (sinf(pitch)*sinf(roll) + cosf(yaw)*cosf(roll)*sinf(pitch)) *roll_i +(cosf(roll)*sinf(pitch)*sinf(yaw)-cosf(yaw)*sinf(roll))      *    pitch_i +  cosf(pitch)*cosf(roll)  *yaw_i;
    return {b_roll, b_pitch, b_yaw};
}