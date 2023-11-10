#include "ES_9_PID.h"

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL &hal;

ES_9_PID::ES_9_PID(float Kp, float Ki, float Kd, float TS, float lowpassPole) :
    kp{ Kp }, ki{ Ki }, kd{ Kd }, Ts { TS }, N { lowpassPole }
{

}
ES_9_PID::ES_9_PID(float Kp, float Ki, float Kd, float TS, float lowpassPole, float OutputSaturation) :
    kp{ Kp }, ki{ Ki }, kd{ Kd }, Ts { TS }, N { lowpassPole }, outputSaturation { OutputSaturation }, saturationEnabled { true }
{

}
    
float ES_9_PID::calculatePIDOutput(float new_measurement)
{
    // hal.console->printf("Terms: %f | %f | %f | %f | %f \n", term1Scale, term2Scale, term3Scale, term4Scale, term5Scale);
    float new_error = input_reference - new_measurement;

    // if (signbit(new_error) != signbit(prev_error.at(0)))
    // {
    //     prev_output.at(1) = 0.f;
    //     prev_output.at(0) = 0.f;
    // }

    float new_output = term1Scale * new_error + term2Scale * prev_error.at(0) + term3Scale * prev_error.at(1) + term4Scale * prev_output.at(0) + term5Scale * prev_output.at(1);
    
    if(saturationEnabled && abs(new_output) > outputSaturation)
    {
        new_output = new_output/abs(new_output) * outputSaturation;
    }

    prev_error.at(1) = prev_error.at(0);
    prev_error.at(0) = new_error;
    prev_output.at(1) = prev_output.at(0);
    prev_output.at(0) = new_output;
    
    
    return new_output;
}

void ES_9_PID::setReference(float new_reference)
{
    input_reference = new_reference;
}

float ES_9_PID::getReference() const
{
    return input_reference;
}