
#include "ES_9_PID.h"
#include "Es_9_Sensor/IMU.h"

class Controller2
{

public:
Controller2(IMU& imu, ES_9_PID& pid_roll_angularRate, ES_9_PID& pid_pitch_angularRate, ES_9_PID& pid_yaw_angularRate);
void loop(); 

private:   
    ES_9_PID& _pid_roll_angularRate;
    ES_9_PID& _pid_pitch_angularRate;
    ES_9_PID& _pid_yaw_angularRate;
    IMU& _imu;
    
    
};





    