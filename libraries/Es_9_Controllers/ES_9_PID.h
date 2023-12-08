#pragma once
#include <array>
#include <cmath>

class ES_9_PID
{
public:
    ES_9_PID(float Kp, float Ki, float Kd, float TS, float lowpassPole);
    ES_9_PID(float Kp, float Ki, float Kd, float TS, float lowpassPole, float OutputSaturation);
    float calculatePIDOutput(float new_measurement);
    void setReference(float new_reference);
    float getReference() const;

private:
    float input_reference { 0.f };

    const float kp { 0.f };
    const float ki { 0.f };
    const float kd { 0.f };
    const float Ts { 0.f };
    const float N { 0.f };
    const bool saturationEnabled { false };
    const float outputSaturation{ 0.f };

    float term1Scale {0};
    float term2Scale {0};
    float term3Scale {0};
    float term4Scale {0};
    float term5Scale {0};
    
    std::array<float, 2> prev_output { 0.f };
    std::array<float, 2> prev_error { 0.f };
};
