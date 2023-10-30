#include "MotorMixing.h"

std::tuple<float, float, float, float> MotorMixing::mix(float torque_roll, float torque_pitch, float torque_yaw, float force)
{
    float omega_m1 = m/(4*kt) * force - Ixx/(4*kt*l_roll) * torque_roll - Iyy/(4*kt*l_pitch) * torque_pitch + Izz/(4*km) * torque_yaw;
    float omega_m2 = m/(4*kt) * force + Ixx/(4*kt*l_roll) * torque_roll + Iyy/(4*kt*l_pitch) * torque_pitch + Izz/(4*km) * torque_yaw;
    float omega_m3 = m/(4*kt) * force + Ixx/(4*kt*l_roll) * torque_roll - Iyy/(4*kt*l_pitch) * torque_pitch - Izz/(4*km) * torque_yaw;
    float omega_m4 = m/(4*kt) * force - Ixx/(4*kt*l_roll) * torque_roll + Iyy/(4*kt*l_pitch) * torque_pitch - Izz/(4*km) * torque_yaw;

    return {omega_m1, omega_m2, omega_m3, omega_m4};
}