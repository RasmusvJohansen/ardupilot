#include "Sensor.h"
#include "AP_HAL/AP_HAL.h"
#include <AP_Baro/AP_Baro.h>

static AP_Baro barometer;

class Barometer : public Sensor
{

public:
    // This can be changed to contain all sensors.
    // Remember sensor_List_stop
    enum class Sensors : int
    {
        Baro1,
        Baro2,
        Sensor_List_stop,
    };
    // This can be changed to contain all measurements.
    // Remeber Measurments_Type_List_Stop
    enum class Measurements : int
    {
        acc_x,
        acc_y,
        Measurements_Type_List_Stop,
    };

    virtual void init() override;
    virtual void updateMeasurements() override;
    std::map<Barometer::Sensors, std::map<Barometer::Measurements, float>> getMeasurements();

    virtual void loop() override;

private:
    const AP_HAL::HAL &hal = AP_HAL::get_HAL();

    std::map<Sensors, std::map<Measurements, float>> sensors;
    std::map<Measurements, float> measurements;
};
