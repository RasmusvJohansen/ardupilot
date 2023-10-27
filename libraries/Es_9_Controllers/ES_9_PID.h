#pragma once
#include <array>

class ES_9_PID
{
public:
    ES_9_PID(float Kp, float Ki, float Kd, float TS);
    float calculatePIDOutput(float new_error);
    void setReference(float new_reference);

private:
    float input_reference { 0.f };
    const float max_allowed_reference_rad { 0.4f };

    const float kp { 0.f };
    const float ki { 0.f };
    const float kd { 0.f };
    const float Ts { 0.f };
    const float N { 100.f };
    
    std::array<float, 2> prev_output { 0.f };
    std::array<float, 2> prev_error { 0.f };
};
