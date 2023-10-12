//
// Simple test for the AP_InertialSensor driver.
//

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialSensor/AP_InertialSensor_Invensense.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS_Dummy.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

static AP_InertialSensor_Invensense ins;
#if HAL_EXTERNAL_AHRS_ENABLED
 static AP_ExternalAHRS eAHRS;
#endif // HAL_EXTERNAL_AHRS_ENABLED

// board specific config
static AP_BoardConfig BoardConfig;
static AP_Int32 log_bitmask;
static AP_Logger logger{log_bitmask};

void setup(void);
void loop(void);

void setup(void)
{
    // setup any board specific drivers
    BoardConfig.init();

    hal.console->begin(115200);
    hal.console->printf("AP_InertialSensor startup...\n");

    ins._init();
}

void loop(void)
{
 
}

const struct AP_Param::GroupInfo        GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

AP_HAL_MAIN();
