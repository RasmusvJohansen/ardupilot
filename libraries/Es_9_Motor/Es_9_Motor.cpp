#include <Es_9_Motor/Es_9_Motor.h>

#include <AP_HAL/AP_HAL.h>
extern const AP_HAL::HAL &hal;

Es_9_Motor::Es_9_Motor(uint8_t NumMotors) : numMotors{NumMotors}
{
}

void Es_9_Motor::armMotors()
{
    hal.rcout->set_freq(0xFF, 490);
    hal.rcout->force_safety_off();
    // Enable the motors in HAL
    for (uint8_t i = 0; i <= numMotors; i++)
    {
        hal.rcout->enable_ch(i);
    }

    // ramp up motor to start sequence
    setPeriod(armPeriod);
    hal.scheduler->delay(500);

    // decrease to finish arming.
    setPeriod(minPeriod);
    hal.scheduler->delay(1000);  //OBS on this, might need to be deleted
    isArmed = true;
}
void Es_9_Motor::disarmMotors()
{
    // Disable the motors in HAL
    for (uint8_t i = 0; i <= numMotors; i++)
    {
        hal.rcout->disable_ch(i);
    }
    hal.rcout->force_safety_on();
    isArmed = false;
}

void Es_9_Motor::setAllMotorPeriod(uint16_t periodMotorOne, uint16_t periodMotorTwo, uint16_t periodMotorThree, uint16_t periodMotorFour)
{
    if (!isArmed)
    {
        return;
    }
    // Check for valid input
    // if((1000 <= periodMotorOne <= 2000 && 1000 <= periodMotorTwo <= 2000 && 1000 <= periodMotorThree <= 2000 && 1000 <= periodMotorFour <= 2000))
    // {
    //     setPeriod(minPeriod);
    //     return;
    // }

    setPeriod(periodMotorOne, periodMotorTwo, periodMotorThree, periodMotorFour);
}

uint16_t Es_9_Motor::convert_float_rads_to_uint_ms(float angular_velocity)
{
    float pulse_width_float = angular_velocity/angular_velocity_scale + period_offset;
    return static_cast<uint16_t>(pulse_width_float);
}

void Es_9_Motor::setAllMotorAngularVelocity(float motorOneAngularVelocity, float motorTwoAngularVelocity, float motorThreeAngularVelocity, float motorFourAngularVelocity)
{
    uint16_t motorOnePeriod = convert_float_rads_to_uint_ms(motorOneAngularVelocity);
    uint16_t motorTwoPeriod = convert_float_rads_to_uint_ms(motorTwoAngularVelocity);
    uint16_t motorThreePeriod = convert_float_rads_to_uint_ms(motorThreeAngularVelocity);
    uint16_t motorFourPeriod = convert_float_rads_to_uint_ms(motorFourAngularVelocity);
    
    setPeriod(motorOnePeriod, motorTwoPeriod, motorThreePeriod, motorFourPeriod);
}

void Es_9_Motor::setPeriod(uint16_t period)
{
    hal.rcout->cork();
    for (int i = 0; i <= numMotors; i++)
    {
        hal.rcout->write(i, period);
    }
    hal.rcout->push();
}
void Es_9_Motor::setPeriod(uint16_t periodMotorOne, uint16_t periodMotorTwo, uint16_t periodMotorThree, uint16_t periodMotorFour)
{
    hal.rcout->cork();
    hal.rcout->write(0, periodMotorOne);
    hal.rcout->write(1, periodMotorTwo);
    hal.rcout->write(2, periodMotorThree);
    hal.rcout->write(3, periodMotorFour);
    hal.rcout->push();
}

bool Es_9_Motor::getIsArmed() const
{
    return isArmed;
}