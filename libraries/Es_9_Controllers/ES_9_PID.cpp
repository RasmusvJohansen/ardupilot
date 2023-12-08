#include "ES_9_PID.h"

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL &hal;

ES_9_PID::ES_9_PID(float Kp, float Ki, float Kd, float TS, float lowpassPole) :
    kp{ Kp }, ki{ Ki }, kd{ Kd }, Ts { TS }, N { lowpassPole }
{  
   if ((kp > 0.f) && (ki >0.f) && (kd > 0.f)) 
    {    
        term1Scale = ((4.f+2.f*N*Ts)*Kp + (2.f*Ts+N*powf(Ts,2.f))*Ki + 4.f*N*Kd)/(4.f+2.f*N*Ts);
        term2Scale = (-8.f*Kp+2.f*N*powf(Ts,2.f)*Ki-8.f*N*Kd)/(4.f+2.f*N*Ts);
        term3Scale = ((4.f-2.f*N*Ts)*Kp+(N*powf(Ts,2.f)-2.f*Ts)*Ki+4.f*N*Kd)/(4.f+2.f*N*Ts);
        term4Scale = (8.f)/(4.f+2.f*N*Ts);
        term5Scale = -(4.f-2.f*N*Ts)/(4.f+2.f*N*Ts);
    }
    else if((kp >0.f) && (kd > 0.f) )
    {
        term1Scale = ((2.f+N*Ts)*Kp+2.f*N*Kd)/(2.f+N*Ts);
        term2Scale = ((N*Ts-2.f)*Kp-2.f*N*Kd)/(2.f+N*Ts);
        term4Scale = -(N*Ts-2.f)/(2.f+N*Ts);
    }

    else if((kp >0.f) && (ki > 0.f))
    {
        term1Scale = kp+ki*(Ts/2.f); 
        term2Scale = ki*(Ts/2.f)-kp;
        term4Scale = 1;
    }

    else if(kp >0.f )
    {
        term1Scale = Kp;
    }



}
ES_9_PID::ES_9_PID(float Kp, float Ki, float Kd, float TS, float lowpassPole, float OutputSaturation) :
    kp{ Kp }, ki{ Ki }, kd{ Kd }, Ts { TS }, N { lowpassPole }, outputSaturation { OutputSaturation }, saturationEnabled { true }
{
    if ((kp > 0.f) && (ki >0.f) && (kd > 0.f)) 
    {    
        term1Scale = ((4.f+2.f*N*Ts)*Kp + (2.f*Ts+N*powf(Ts,2.f))*Ki + 4.f*N*Kd)/(4.f+2.f*N*Ts);
        term2Scale = (-8.f*Kp+2.f*N*powf(Ts,2.f)*Ki-8.f*N*Kd)/(4.f+2.f*N*Ts);
        term3Scale = ((4.f-2.f*N*Ts)*Kp+(N*powf(Ts,2.f)-2.f*Ts)*Ki+4.f*N*Kd)/(4.f+2.f*N*Ts);
        term4Scale = (8.f)/(4.f+2.f*N*Ts);
        term5Scale = -(4.f-2.f*N*Ts)/(4.f+2.f*N*Ts);
    }
    else if((kp >0.f) && (kd > 0.f) )
    {
        term1Scale = ((2.f+N*Ts)*Kp+2.f*N*Kd)/(2.f+N*Ts);
        term2Scale = ((N*Ts-2.f)*Kp-2.f*N*Kd)/(2.f+N*Ts);
        term4Scale = -(N*Ts-2.f)/(2.f+N*Ts);
    }

    else if((kp >0.f) && (ki > 0.f))
    {
        term1Scale = (kp+ki*(Ts/2.f)); 
        term2Scale = (ki*(Ts/2.f)-kp);
        term4Scale = 1;
    }

    else if(kp >0.f )
    {
        term1Scale = Kp;
    }
}
    
float ES_9_PID::calculatePIDOutput(float new_measurement)
{
    
        float new_error = input_reference - new_measurement;

        // if (saturationEnabled && new_error/abs(new_error) != prev_error.at(0)/abs(prev_error.at(0)))
        // {
        //     prev_output.at(0) = 0.f;
        //     prev_output.at(1) = 0.f;
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