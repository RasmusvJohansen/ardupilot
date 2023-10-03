/*
  simple test of RC output interface
  Attention: If your board has safety switch,
  don't forget to push it to enable the PWM output.
 */

#include <AP_HAL/AP_HAL.h>

// we need a boardconfig created so that the io processor's enable
// parameter is available
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>
AP_BoardConfig BoardConfig;
#endif

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup (void)
{
    hal.rcout->force_safety_off();
    

    hal.console->printf("Starting AP_HAL::RCOutput test\n");
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    BoardConfig.init();
#endif
    hal.rcout->enable_ch(0);
    hal.rcout->set_freq(0xFF, 490);

    hal.scheduler->delay(1000);
    for(int j=1000; j<=1500; j++){
        hal.rcout->write(0,j);
        hal.scheduler->delay(5);
    }
    for(int j=1500; j>=1000; j--){
        hal.rcout->write(0,j);
        hal.scheduler->delay(5);
    }
}

void loop (void)
{
    hal.rcout->write(0, 1500);
    hal.scheduler->delay(5);
}

AP_HAL_MAIN();
