#include "MotorMixing.h"

std::tuple<float, float, float, float> MotorMixing::mix(float torque_roll, float torque_pitch, float torque_yaw, float force)
{
    float omega_m1 = m/(4*kt) - Ixx/(4*kt*l_roll) - Iyy/(4*kt*l_pitch) + Izz/(4*km);
    float omega_m2 = m/(4*kt) + Ixx/(4*kt*l_roll) + Iyy/(4*kt*l_pitch) + Izz/(4*km);
    float omega_m3 = m/(4*kt) + Ixx/(4*kt*l_roll) - Iyy/(4*kt*l_pitch) - Izz/(4*km);
    float omega_m4 = m/(4*kt) - Ixx/(4*kt*l_roll) + Iyy/(4*kt*l_pitch) - Izz/(4*km);

    return {omega_m1, omega_m2, omega_m3, omega_m4};
}