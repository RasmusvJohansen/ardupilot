#include "Sensor.h"
#include "AP_HAL/AP_HAL.h"
#include "AP_InertialSensor/AP_InertialSensor.h"

class IMU : public Sensor
{

public:
    // This can be changed to contain all sensors.
    // Remember sensor_List_stop
    enum class Sensors : int
    {
        IMU1,
        IMU2,
        IMU3,
        Sensor_List_stop,
    };
    // This can be changed to contain all measurements.
    // Remeber Measurments_Type_List_Stop
    enum class Measurements : int
    {
        gyr_x,
        gyr_y,
        gyr_z,
        acc_roll,
        acc_pitch,
        Measurements_Type_List_Stop,
    };

    virtual void init() override;
    virtual void updateMeasurements() override;
    std::map<IMU::Sensors, std::map<IMU::Measurements, float>> getMeasurements();

    virtual void loop() override;

private:
    Vector3f acc;
    Vector3f gyr;

    int NrOfAccMeas{ 3 }; // noget

    const AP_HAL::HAL &hal = AP_HAL::get_HAL();

    std::map<Sensors, std::map<Measurements, float>> sensors;
    std::map<Measurements, float> measurements;
};
