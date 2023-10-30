#pragma once
#include <array>
#include <cmath>

class ES_9_PID
{
public:
    ES_9_PID(float Kp, float Ki, float Kd, float TS);
    float calculatePIDOutput(float new_measurement);
    void setReference(float new_reference);
    float getReference() const;

private:
    float input_reference { 0.f };

    const float kp { 0.f };
    const float ki { 0.f };
    const float kd { 0.f };
    const float Ts { 0.f };
    const float N { 100.f };

    const float term1Scale {((4.f+2.f*N*Ts)*kp + (2.f*Ts+N*powf(Ts,2.f))*ki + 4.f*N*kd) / (4.f+2.f*N*Ts)};
    const float term2Scale {(-8.f*kp+2.f*N*powf(Ts,2.f)*ki-8.f*N*kd) / (4.f+2.f*N*Ts)};
    const float term3Scale {((4.f-2.f*N*Ts)*kp+(N*powf(Ts,2.f)-2.f*Ts)*ki+4.f*N*kd) / (4.f+2.f*N*Ts)};
    const float term4Scale {(8.f) / (4.f+2.f*N*Ts)};
    const float term5Scale {-(4.f-2.f*N*Ts) / (4.f+2.f*N*Ts)};
    
    std::array<float, 2> prev_output { 0.f };
    std::array<float, 2> prev_error { 0.f };
};
