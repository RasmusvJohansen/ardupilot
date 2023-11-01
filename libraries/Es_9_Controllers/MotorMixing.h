#include <tuple>

class MotorMixing
{
public:
    std::tuple<float, float, float, float> mix(float torque_roll, float torque_pitch, float torque_yaw, float force);
private:
    float kt { 0.011f };
    float km { 0.000327f };
    float m { 1.513f };
    float Ixx { 0.0185f };
    float Iyy { 0.0201f };
    float Izz { 0.0286f };
    float l_roll { 0.175f };
    float l_pitch { 0.175f };
};
