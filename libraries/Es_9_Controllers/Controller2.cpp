#include "Controller2.h"


Controller2::Controller2(IMU& imu, ES_9_PID& pid_roll_angularRate, ES_9_PID& pid_pitch_angularRate, ES_9_PID& pid_yaw_angularRate) : 
_imu{imu}, _pid_roll_angularRate{pid_roll_angularRate}, _pid_pitch_angularRate{pid_roll_angularRate}, _pid_yaw_angularRate{pid_yaw_angularRate}
{
    
}    
