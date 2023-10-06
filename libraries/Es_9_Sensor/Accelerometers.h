#include "Sensor.h"
class Accelerometers: public Sensor
{

public:
    //virtual void init() override; 
    //virtual void updateMeasurement() override;
    virtual float getMeasurements() override;

private:
    /* data */
    
};
