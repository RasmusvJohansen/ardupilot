#include "Controller.h"

Controller::Controller(Complementary_Filter& complementary_filter, Barometer& barometer, ES_9_PID& pid_roll, ES_9_PID& pid_pitch, ES_9_PID& pid_yaw, ES_9_PID& pid_altitude, Es_9_Motor& motor_controller) :
    _complementary_filter{ complementary_filter }, _barometer{ barometer }, _pid_roll{ pid_roll }, _pid_pitch{ pid_pitch }, _pid_yaw{ pid_yaw }, _pid_altitude{ pid_altitude }, _motorController{ motor_controller }
{

}

void Controller::loop()
{
    float torque_roll = _pid_roll.calculatePIDOutput(_complementary_filter.getRoll());
    float torque_pitch = _pid_pitch.calculatePIDOutput(_complementary_filter.getPitch());
    float torque_yaw = _pid_yaw.calculatePIDOutput(_complementary_filter.getYaw());
    // float force = _pid_altitude.calculatePIDOutput(_barometer.getMeasurements().at(Barometer::Sensors::baro_1).at(Barometer::Measurements::baro_altitude));
    float force { 0 };
    float omega_m1 {0.f}; 
    float omega_m2 {0.f};
    float omega_m3 {0.f};
    float omega_m4 {0.f};
    std::tie(omega_m1, omega_m2, omega_m3, omega_m4) = motor_mixing.mix(torque_roll, torque_pitch, torque_yaw, force);

    _motorController.setAllMotorAngularVelocity(omega_m1, omega_m2, omega_m3, omega_m4);
}