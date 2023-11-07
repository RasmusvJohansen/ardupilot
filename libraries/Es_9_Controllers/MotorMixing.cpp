#include "MotorMixing.h"

std::tuple<float, float, float, float> MotorMixing::mix(float u_roll, float u_pitch, float u_yaw, float u_z)
{
    float omega_m1 = 1.f/(4.f) * u_z - 1.f/(4.f) * u_roll - 1.f/(4.f) * u_pitch + 1.f/(4.f) * u_yaw;
    float omega_m2 = 1.f/(4.f) * u_z + 1.f/(4.f) * u_roll + 1.f/(4.f) * u_pitch + 1.f/(4.f) * u_yaw;
    float omega_m3 = 1.f/(4.f) * u_z + 1.f/(4.f) * u_roll - 1.f/(4.f) * u_pitch - 1.f/(4.f) * u_yaw;
    float omega_m4 = 1.f/(4.f) * u_z - 1.f/(4.f) * u_roll + 1.f/(4.f) * u_pitch - 1.f/(4.f) * u_yaw;

    return {omega_m1, omega_m2, omega_m3, omega_m4};
}