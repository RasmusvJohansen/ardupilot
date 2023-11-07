#include "Controller.h"

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL &hal;

Controller::Controller(Complementary_Filter& complementary_filter, IMU& imu, Barometer& barometer, Es_9_Motor& motor_controller, GPS_fake& gps, fake_measurement& fake_measurement, ES_9_PID& pid_roll_angularRate, ES_9_PID& pid_pitch_angularRate, ES_9_PID& pid_yaw_angularRate, ES_9_PID& pid_roll, ES_9_PID& pid_pitch, ES_9_PID& pid_yaw, ES_9_PID& pid_altitude) :
    _complementary_filter{ complementary_filter }, _imu{ imu }, _barometer{ barometer }, _motorController{ motor_controller }, _gps{ gps }, _fake_measurement { fake_measurement }, _pid_roll_angularRate{ pid_roll_angularRate }, _pid_pitch_angularRate{ pid_pitch_angularRate }, _pid_yaw_angularRate{ pid_yaw_angularRate }, _pid_roll{ pid_roll }, _pid_pitch{ pid_pitch }, _pid_yaw{ pid_yaw }, _pid_altitude{ pid_altitude }
{

}

void Controller::InnerLoop()
{
    if(!runController())
    {
        return;
    }
    float interial_rate_roll, interial_rate_pitch, interial_rate_yaw { 0.f };
    std::tie(interial_rate_roll, interial_rate_pitch, interial_rate_yaw) = body_angularRate_to_inertial_angular_rate(_imu.getMeasurements().at(IMU::Sensors::IMU1).at(IMU::Measurements::gyr_x),_imu.getMeasurements().at(IMU::Sensors::IMU1).at(IMU::Measurements::gyr_y),_imu.getMeasurements().at(IMU::Sensors::IMU1).at(IMU::Measurements::gyr_z));

    u_roll = _pid_roll_angularRate.calculatePIDOutput(interial_rate_roll);
    u_pitch = _pid_pitch_angularRate.calculatePIDOutput(interial_rate_pitch);
    u_yaw = _pid_yaw_angularRate.calculatePIDOutput(interial_rate_yaw);

    // hal.console->printf("Body: %f | %f | %f | Inertial: %f | %f | %f \n",_imu.getMeasurements().at(IMU::Sensors::IMU1).at(IMU::Measurements::gyr_x),_imu.getMeasurements().at(IMU::Sensors::IMU1).at(IMU::Measurements::gyr_y),_imu.getMeasurements().at(IMU::Sensors::IMU1).at(IMU::Measurements::gyr_z),interial_rate_roll, interial_rate_pitch, interial_rate_yaw);

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

    float reference_angularRate_roll = _pid_roll.calculatePIDOutput(roll);
    float reference_angularRate_pitch = _pid_pitch.calculatePIDOutput(pitch);
    float reference_angularRate_yaw = _pid_yaw.calculatePIDOutput(yaw);

    _pid_roll_angularRate.setReference(reference_angularRate_roll);
    _pid_pitch_angularRate.setReference(reference_angularRate_pitch);
    _pid_yaw_angularRate.setReference(reference_angularRate_yaw);

    u_z = _pid_altitude.calculatePIDOutput(z);

    adjustOutput();
}
void Controller::OuterLoop()
{
    
}

void Controller::adjustOutput()
{
    float omega_m1, omega_m2, omega_m3, omega_m4 { 0.f };
    std::tie(omega_m1, omega_m2, omega_m3, omega_m4) = motor_mixing.mix(u_roll, u_pitch, u_yaw, u_z);

    hal.console->printf("F: %f| R: %f| P: %f| Y: %f| m1 %f| m2 %f| m3 %f| m4 %f| \n",u_z, u_roll,u_pitch, u_yaw, omega_m1,omega_m2, omega_m3, omega_m4);
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

std::tuple<float, float, float> Controller::body_angularRate_to_inertial_angular_rate(float body_rate_x, float body_rate_y, float body_rate_z)
{
    float roll, pitch, yaw, z {0.f};
    std::tie(roll, pitch, yaw, z) = _fake_measurement.getMeasurement();
     
    float inertial_rate_roll =  1 * body_rate_x + tanf(pitch)*sinf(roll) *  body_rate_y - tanf(pitch)*cosf(roll) *  body_rate_z; 
    float inertial_rate_pitch = 0 * body_rate_x + cosf(roll) *              body_rate_y + sinf(roll) *              body_rate_z;
    float inertial_rate_yaw =   0 * body_rate_x - sinf(roll)/cosf(pitch) *  body_rate_y + cosf(roll)/cosf(pitch) *  body_rate_z;

    return {inertial_rate_roll, inertial_rate_pitch, inertial_rate_yaw};
}