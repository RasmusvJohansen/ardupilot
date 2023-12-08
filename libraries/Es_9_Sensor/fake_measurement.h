#pragma once

#include <array>
#include <tuple>
#include "math.h"
class fake_measurement
{
public:

    std::tuple<float, float, float, float> getMeasurement() const;
    std::tuple<float,float,float> getVelocity() const;
    std::tuple<float, float, float> getPosition() const;
    std::tuple<float, float, float, float, float, float> getMeasurementForLogging() const;
    void setMeasurement(float roll, float pitch, float yaw, float Altitude, float X, float Y);
private:
    std::array<float, 3> attitude { 0.f };
    float altitude { 0.f };
    float x,y { 0.f };
    float v_x,v_y,v_z {0.f};
};