#include <Es_9_Motor/Es_9_Motor.h>

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL &hal;

Es_9_Motor::Es_9_Motor(int NumMotors) : numMotors{NumMotors}
{
}

bool Es_9_Motor::armMotors()
{
    hal.rcout->force_safety_off();
    hal.rcout->set_freq(0xFF, 490);
    for (int i = 0; i <= numMotors; i++)
    {
        hal.rcout->enable_ch(i);
    }

    // ramp up motor to start sequence
    for (int i = 0; i <= numMotors; i++)
    {
        setPWM(1100, i);
    }
    hal.scheduler->delay(500);

    // decrease to finish arming.
    for (int i = 0; i <= numMotors; i++)
    {
        setPWM(1000, i);
    }
    hal.scheduler->delay(1000);
    // change to true/false if armed?
    return true;
}

void Es_9_Motor::setPWM(int PWM, int motor_channel)
{
    hal.rcout->write(motor_channel, PWM);
}

int Es_9_Motor::getPWM(int motor_channel) const
{
    return 0;
}