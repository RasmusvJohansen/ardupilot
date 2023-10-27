#include "ES_9_PID.h"
#include <cmath>

ES_9_PID::ES_9_PID(float Kp, float Ki, float Kd, float TS) :
    kp{ Kp }, ki{ Ki }, kd{ Kd }, Ts { TS }
{

}

float ES_9_PID::calculatePIDOutput(float new_error){
    
    float term1Scale {((4.f+2.f*N*Ts)*kp + (2.f*Ts+N*powf(Ts,2.f))*ki + 4.f*N*kd) / (4.f+2.f*N*Ts)};
    float term2Scale {(-8.f*kp+2.f*N*powf(Ts,2.f)*ki-8.f*N*kd) / (4.f+2.f*N*Ts)};
    float term3Scale {((4.f-2.f*N*Ts)*kp+(N*powf(Ts,2.f)-2.f*Ts)*ki+4.f*N*kd) / (4.f+2.f*N*Ts)};
    float term4Scale {(8.f) / (4.f+2.f*N*Ts)};
    float term5Scale {-(4.f-2.f*N*Ts) / (4.f+2.f*N*Ts)};

    float newOutput = term1Scale * new_error + term2Scale * prev_error[0] + term3Scale * prev_error[1] + term4Scale * prev_output[0] + term5Scale * prev_output[1];
    prev_error[1] = prev_error[0];
    prev_error[0] = new_error;
    prev_output[1] = prev_output[0];
    prev_error[0] = newOutput;

    return newOutput;
}

void ES_9_PID::setReference(float new_reference){
    if (abs(new_reference) >= max_allowed_reference_rad) {
        input_reference = new_reference;
    }
}