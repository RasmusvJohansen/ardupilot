#pragma once

#include "Es_9_Sensor/IMU.h"
#include "Es_9_Sensor/Barometer.h"
#include "Es_9_Sensor/Magnetometer.h"
#include "Es_9_Sensor/GPS_fake.h"
#include "Es_9_Sensor/fake_measurement.h"

#include <tuple>

class Kalman
{
public:
    Kalman(IMU& imu, Barometer& barometer, Magnetometer& magnetometer, GPS_fake& gps, fake_measurement& fake_measurement);
    
    void Predict();
    void Update();
    

private:
    IMU& _imu;
    Barometer& _barometer;
    Magnetometer& _magnetometer;
    GPS_fake& _gps;
    fake_measurement& _fake_measurement;

};
