#include "Controller.h"

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL &hal;

Controller::Controller(Complementary_Filter& complementary_filter, Barometer& barometer, ES_9_PID& pid_roll, ES_9_PID& pid_pitch, ES_9_PID& pid_yaw, ES_9_PID& pid_altitude, Es_9_Motor& motor_controller, GPS_fake& gps, fake_measurement& fake_measurement) :
    _complementary_filter{ complementary_filter }, _barometer{ barometer }, _pid_roll{ pid_roll }, _pid_pitch{ pid_pitch }, _pid_yaw{ pid_yaw }, _pid_altitude{ pid_altitude }, _motorController{ motor_controller }, _gps{ gps }, _fake_measurement { fake_measurement }
{

}


void Controller::loop()
{
    if(!_motorController.getIsFlying())
    {
        if(_motorController.getIsArmed())
        {
            _motorController.setAllMotorPeriod(1100,1100,1100,1100);
        }
        return;
    }
    float _roll, _pitch, _yaw, _z {0.f};
    std::tie(_roll, _pitch, _yaw, _z) = _fake_measurement.getMeasurement();
    
    // float torque_roll = _pid_roll.calculatePIDOutput(_roll);
    float torque_pitch = _pid_pitch.calculatePIDOutput(_pitch);
    // float torque_yaw = _pid_yaw.calculatePIDOutput(_yaw);
    // float force = _pid_altitude.calculatePIDOutput(_z);
    
    // hal.console->printf("%f|%f|%f|%f \n",_roll,_pitch,_yaw, _z);
    // hal.console->printf("%f|%f \n",_pitch,torque_pitch);

    float omega_m1 { 0.f }; 
    float omega_m2 { 0.f };
    float omega_m3 { 0.f };
    float omega_m4 { 0.f };
    std::tie(omega_m1, omega_m2, omega_m3, omega_m4) = motor_mixing.mix(0, torque_pitch, 0, 0);
    // std::tie(omega_m1, omega_m2, omega_m3, omega_m4) = motor_mixing.mix(torque_roll, torque_pitch, torque_yaw, force);
    hal.console->printf("F: %f| TR: %f| TP: %f| TY: %f| m1 %f| m2 %f| m3 %f| m4 %f| \n",0.f, 0.f,torque_pitch, 0.f, omega_m1,omega_m2, omega_m3, omega_m4);
    // hal.console->printf("F: %f| TR: %f| TP: %f| TY: %f| m1 %f| m2 %f| m3 %f| m4 %f|",force, torque_roll,torque_pitch, torque_yaw, omega_m1,omega_m2, omega_m3, omega_m4);

    _motorController.setAllMotorAngularVelocity(omega_m1 + input_linearisation_rads, omega_m2 + input_linearisation_rads, omega_m3 + input_linearisation_rads, omega_m4 + input_linearisation_rads);
}