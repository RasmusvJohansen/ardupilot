#include "IMU.h"

void IMU::init()
{
    // add every type of measurement to the measurements map
    for (int measurement = 0; measurement != static_cast<int>(Measurements::Measurements_Type_List_Stop); measurement++)
    {
        measurements.insert({IMU::Measurements(measurement), 0});
    }

    // add every type of measurment to each sensor.
    for (int sensor = 0; sensor != static_cast<int>(Sensors::Sensor_List_stop); sensor++)
    {
        sensors.insert({IMU::Sensors(sensor), measurements});
    }

    // Init sensors here
    // The IMU is already initialised in system.cpp line 230. Check loop rate 

    // Accel calibration in ap_vehicle
}

void IMU::updateMeasurements()
{
    //Vector3f accel;
    //Vector3f gyro;

    // Clear out any existing samples from imu and update gyro and accel values from backends
    imu.update();

    // Goes through each senor, and measurement type and update each values with the current measurement.
    for (int sensor = 0; sensor != static_cast<int>(Sensors::Sensor_List_stop); sensor++)
    {
        // Get accel and gyro measurements
        //accel = imu.get_accel(sensor);
        //gyro = imu.get_gyro(sensor);
        const Vector3f &accel = imu.get_accel(sensor);
        const Vector3f &gyro = imu.get_gyro(sensor);

        for (int measurement = 0; measurement != static_cast<int>(Measurements::Measurements_Type_List_Stop); measurement++)
        {
            sensors.at(IMU::Sensors(sensor)).at(IMU::Measurements(measurement)) = (measurement < 3) ? accel[measurement] : gyro[measurement - 3]; // here it should get the corresponding measurement for the sensor and measurement type
        }
    }
}

std::map<IMU::Sensors, std::map<IMU::Measurements, float>> IMU::getMeasurements()
{
    return {sensors};
}

void IMU::loop()
{
    // main loop for the sensors should contain, updateMeasurements and any transformation which should be applied to the measurements.

    //hal.console->printf("%f", sensors.at(IMU::Sensors::Sensor1).at(IMU::Measurements::acc_x));
    //updateMeasurements();
    //hal.console->printf("%f", sensors.at(IMU::Sensors::Sensor1).at(IMU::Measurements::acc_x));
}