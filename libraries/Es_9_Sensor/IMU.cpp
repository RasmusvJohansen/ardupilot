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
}

void IMU::updateMeasurements()
{

    // Clear out any existing samples from imu and update gyro and accel values from backends
    AP::ins().update();

    // Goes through each senor, and measurement type and update each values with the current measurement.
    for (int sensor = 0; sensor != static_cast<int>(Sensors::Sensor_List_stop); sensor++)
    {
        // Get accel and gyro measurements
        //accel = AP::ins().get_accel(sensor);
        //gyro = AP::ins().get_gyro(sensor);

        acc = AP::ins().get_accel(sensor);
        gyr = AP::ins().get_gyro(sensor);

        for (int measurement = 0; measurement != static_cast<int>(Measurements::Measurements_Type_List_Stop); measurement++)
        {
            // Measure acceleration and angular velocity in m/s^2 and rad/s
            sensors.at(IMU::Sensors(sensor)).at(IMU::Measurements(measurement)) = (measurement < NrOfAccMeas) ? acc[measurement] : gyr[measurement - NrOfAccMeas]; // here it should get the corresponding measurement for the sensor and measurement type
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

    updateMeasurements();
    //hal.console->printf("X: %.2f ", sensors.at(IMU::Sensors::IMU1).at(IMU::Measurements::acc_x));
    //hal.console->printf("Y: %.2f ", sensors.at(IMU::Sensors::IMU1).at(IMU::Measurements::acc_y));
    //hal.console->printf("Z: %.2f ", sensors.at(IMU::Sensors::IMU1).at(IMU::Measurements::acc_z));
    //hal.console->printf("x: %.2f ", sensors.at(IMU::Sensors::IMU1).at(IMU::Measurements::gyr_x));
    //hal.console->printf("x: %.2f ", sensors.at(IMU::Sensors::IMU1).at(IMU::Measurements::gyr_y));
    //hal.console->printf("x: %.2f\n", sensors.at(IMU::Sensors::IMU1).at(IMU::Measurements::gyr_z));
}