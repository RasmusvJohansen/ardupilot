#include "Controller.h"

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL &hal;

Controller::Controller(Complementary_Filter& complementary_filter, Barometer& barometer, ES_9_PID& pid_roll, ES_9_PID& pid_pitch, ES_9_PID& pid_yaw, ES_9_PID& pid_altitude, Es_9_Motor& motor_controller, GPS_fake& gps) :
    _complementary_filter{ complementary_filter }, _barometer{ barometer }, _pid_roll{ pid_roll }, _pid_pitch{ pid_pitch }, _pid_yaw{ pid_yaw }, _pid_altitude{ pid_altitude }, _motorController{ motor_controller }, _gps{ gps }
{

}

void Controller::loop()
{
    if(!_motorController.getIsFlying())
    {
        return;
    }
    float x, y, z {0.f};
    float torque_roll = _pid_roll.calculatePIDOutput(_complementary_filter.getRoll());
    float torque_pitch = _pid_pitch.calculatePIDOutput(_complementary_filter.getPitch());
    float torque_yaw = _pid_yaw.calculatePIDOutput(_complementary_filter.getYaw());
    // float torque_yaw = _pid_yaw.calculatePIDOutput(0.f);
    std::tie(x, y, z) = _gps.getPosition();
    float force = _pid_altitude.calculatePIDOutput(z);
    
// hal.console->printf("%f|%f|%f|%f \n",_complementary_filter.getRoll(),_complementary_filter.getPitch(),_complementary_filter.getYaw(), z);

    float omega_m1 { 0.f }; 
    float omega_m2 { 0.f };
    float omega_m3 { 0.f };
    float omega_m4 { 0.f };
    std::tie(omega_m1, omega_m2, omega_m3, omega_m4) = motor_mixing.mix(torque_roll, torque_pitch, 0, force);
    hal.console->printf("F: %f| TR: %f| TP: %f| TY: %f| m1 %f| m2 %f| m3 %f| m4 %f| \n",force, torque_roll,torque_pitch, torque_yaw, omega_m1,omega_m2, omega_m3, omega_m4);
    // hal.console->printf("%f\n", _barometer.getMeasurements().at(Barometer::Sensors::baro_1).at(Barometer::Measurements::baro_altitude));
    // hal.console->printf("F: %f| TR: %f| TP: %f| TY: %f| m1 %f| m2 %f| m3 %f| m4 %f|",force, torque_roll,torque_pitch, torque_yaw, omega_m1,omega_m2, omega_m3, omega_m4);

    _motorController.setAllMotorAngularVelocity(omega_m1 + input_linearisation_rads, omega_m2 + input_linearisation_rads, omega_m3 + input_linearisation_rads, omega_m4 + input_linearisation_rads);
}