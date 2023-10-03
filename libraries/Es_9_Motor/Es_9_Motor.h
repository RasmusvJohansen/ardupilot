
// This class contains all related functionality to motors and and access to them.

#ifndef ES_9_MOTORS_H
#define ES_9_MOTORS_H

// #include <AP_HAL/AP_HAL.h>

class Es_9_Motor
{
private:
    bool isArmed;
    int numMotors;

public:
    Es_9_Motor(int NumMotors);
    bool armMotors();
    void setPWM(int PWM, int motor_channel);
    int getPWM(int motor_channel) const;
};

#endif