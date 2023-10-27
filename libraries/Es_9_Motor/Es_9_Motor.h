
// This class contains all related functionality to motors and and access to them.

#ifndef ES_9_MOTORS_H
#define ES_9_MOTORS_H

// #include <AP_HAL/AP_HAL.h>
#include <stdint.h>

class Es_9_Motor
{
private:
    bool isArmed;
    uint8_t numMotors;
    uint16_t minPeriod { 1000 };
    uint16_t maxPeriod { 1000 };
    uint16_t armPeriod { 1100 };
    float angular_velocity_scale { 1.059f };
    float period_offset { 1000.f };
    
    void setPeriod(uint16_t period);
    void setPeriod(uint16_t periodMotorOne, uint16_t periodMotorTwo, uint16_t periodMotorThree, uint16_t periodMotorFour);
    uint16_t convert_float_rads_to_uint_ms(float angular_velocity);
public:
    Es_9_Motor(uint8_t NumMotors);
    void armMotors();
    void disarmMotors();
    void setAllMotorPeriod(uint16_t periodMotorOne, uint16_t periodMotorTwo, uint16_t periodMotorThree, uint16_t periodMotorFour);
    void setAllMotorAngularVelocity(float motorOneAngularVelocity, float motorTwoAngularVelocity, float motorThreeAngularVelocity, float motorFourAngularVelocity);
    bool getIsArmed() const;
};

#endif