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
    // The IMU is already initialised in system.cpp line 230. Check loop rate (maybe 400?)
    AP::ins().init(400);

    // Accel calibration in ap_vehicle
    // Gyro calibration in init(), check in library for how to disable
}

void IMU::updateMeasurements()
{

    // Clear out any existing samples from imu and update gyro and accel values from backends
    AP::ins().update();

    // Goes through each sensor, and measurement type and update each values with the current measurement.
    for (int sensor = 0; sensor != static_cast<int>(Sensors::Sensor_List_stop); sensor++)
    {
        // Get accel and gyro measurements
        gyr = AP::ins().get_gyro(sensor);
        acc = AP::ins().get_accel(sensor);

        // Start angular velocity measurements
        // Measure angular velocity in and rad/s
        sensors.at(IMU::Sensors(sensor)).at(IMU::Measurements::gyr_x) = gyr.x; // here it should get the corresponding measurement for the sensor and measurement type
        sensors.at(IMU::Sensors(sensor)).at(IMU::Measurements::gyr_y) = -gyr.y; // flip rotation around y axis
        sensors.at(IMU::Sensors(sensor)).at(IMU::Measurements::gyr_z) = -gyr.z;

        // acc.z is negative due to the imus z-axis being positive downwards
        sensors.at(IMU::Sensors(sensor)).at(IMU::Measurements::acc_x) = acc.x; // here it should get the corresponding measurement for the sensor and measurement type
        sensors.at(IMU::Sensors(sensor)).at(IMU::Measurements::acc_y) = -acc.y; // flip rotation around y axis
        sensors.at(IMU::Sensors(sensor)).at(IMU::Measurements::acc_z) = -acc.z;
    }
}

std::map<IMU::Sensors, std::map<IMU::Measurements, float>> IMU::getMeasurements()
{
    return {sensors};
}