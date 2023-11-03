#include "AP_Math/vectorN.h"
#include "AP_Math/matrixN.h"


class SEKF
{
public:
    /* Skriv funktioner som viser værdier */
     void calculateStateEstimate()
    {
        
    }


private:
    /* Skriv variabler, matricer, vektorer og funktioner her */
    static const int num_of_states{12};
    static const int num_of_inputs{4};
    static const int time_step{1};
    static const float gravity = 9.82;

    VectorN<float,num_of_states>k;
    VectorN<float,num_of_states>w;

    float roll, pitch, yaw, x_inertia, y_inertia, z_inertia, mass, thrust_constant, roll_vel, pitch_vel, yaw_vel, ang_velo_m1, ang_velo_m2, ang_velo_m3, ang_velo_m4;

    //float torque_roll = cosf(roll)*sinf(roll)*(y_inertia-z_inertia)*powf(pitch_vel,2) + (powf(cosf(roll),2)-powf(sinf(roll),2))*(y_inertia-z_inertia) *pitch_vel *yaw_vel*cosf(pitch)-cosf(roll)*sinf(roll)*(y_inertia-z_inertia)*powf(yaw,2)*powf(cosf(pitch),2) + x_inertia*roll_vel*cosf(roll)*pitch_vel + x_inertia*roll_acc + x_inertia*yaw_acc*sinf(roll);
    //float torque_pitch = y_inertia*pitch_acc*powf(cosf(roll),2)-(yaw_vel*(roll_vel*cosf(pitch)*(cosf(powf(roll,2))-sinf(powf(roll,2)))-pitch_vel*cosf(roll)*sin(pitch)*sin(roll))+yaw_acc*cosf(pitch)*cosf(roll)*sinf(roll))*(y_inertia-z_inertia)+z_inertia*pitch_acc*powf(sinf(roll),2)-x_inertia*yaw_vel*roll_vel*cosf(pitch)+powf(yaw_vel,2)*cosf(pitch)*sin(pitch)*(z_inertia*powf(cosf(roll),2)+y_inertia*powf(sinf(roll),2)-x_inertia)-2*y_inertia*pitch_vel*roll_vel*cosf(roll)*sinf(roll)+2*z_inertia_pitch_vel*roll_vel*sinf(roll*sinf(roll))-pitch_vel*yaw_vel*cosf(roll)*sinf(pitch)*sinf(roll)*(y_inertia-z_inertia);
    //float torque_yaw = yaw_acc * (z_inertia*powf(cosf(pitch),2)*powf(cosf(roll),2)+y_inertia*powf(cosf(pitch),2)*powf(sinf(roll),2)+x_inertia*powf(sinf(pitch),2))-yaw_vel*(2*z_inertia*pitch_vel*cosf(pitch)*powf(cosf(roll),2)*sinf(pitch)-2*x_inertia*pitch_vel*cosf(pitch)*sinf(pitch)-2*);
    
    float torque_roll = 1 ;
    float torque_pitch = 1 ;
    float torque_yaw = 1 ;
    /*
    float roll_acc = -(cosf(roll)*sinf(roll)*(y_inertia-z_inertia)*powf(pitch_vel,2)+(powf(cosf(roll),2)-powf(sinf(roll),2))*(y_inertia-z_inertia)*pitch_vel*yaw_vel*cosf(yaw)-cosf(roll)*sinf(roll)*(y_inertia-z_inertia)*powf(yaw_acc,2)*powf(cosf(pitch),2)+x_inertia*roll_acc*cosf(roll)*yaw_vel-torque_roll+x_inertia*yaw_acc*sinf(roll))/x_inertia;
    float pitch_acc = (torque_pitch+(yaw_vel*(roll_vel*cosf(pitch)*(powf(cosf(roll),2)-powf(sinf(roll),2))-pitch_vel*cosf(roll)*sinf(pitch)*sinf(roll))+yaw_acc*cosf(pitch)*cosf(roll)*sinf(roll))*(y_inertia-z_inertia)+x_inertia*yaw_vel*roll_vel*cosf(pitch)-powf(yaw_vel,2)*cosf(pitch)*sinf(pitch)*(z_inertia*powf(cosf(roll),2)+y_inertia*powf(sinf(roll),2)-x_inertia)+2*y_inertia*pitch_vel*roll_vel*cosf(roll)*sinf(roll)-2*z_inertia*pitch_vel*roll_vel*cosf(roll)*sinf(roll)+pitch_vel*yaw_vel*cosf(roll)*sinf(pitch)*(y_inertia-z_inertia))/(y_inertia*powf(cosf(roll),2)+z_inertia*powf(sinf(roll),2));
    float yaw_acc = (torque_yaw + yaw_vel*(2*z_inertia*pitch_vel*cosf(pitch)*powf(cosf(roll),2)*sinf(pitch)-2*x_inertia*pitch_vel*cosf(pitch)*sinf(pitch)-2*y_inertia*roll_vel*powf(cosf(pitch),2)*cosf(roll)*sinf(roll)+2*z_inertia*roll_vel*powf(cosf(pitch),2)*cosf(roll)*sinf(roll)+2*y_inertia*pitch_vel*cosf(pitch)*sinf(pitch)*powf(sinf(pitch),2))-x_inertia*roll_acc*sinf(pitch)-x_inertia*pitch_vel*roll_vel*cosf(pitch)-powf(pitch_vel,2)*cosf(roll)*sinf(pitch)*sinf(roll)*(y_inertia-z_inertia)+pitch_vel*roll_vel*cosf(pitch)*powf(cosf(roll),2)*(y_inertia-z_inertia)-pitch_vel*roll_vel*cosf(pitch)*powf(sinf(roll),2)+pitch_acc*cosf(roll)*cosf(roll)*sinf(roll)*(y_inertia-z_inertia))/(z_inertia*powf(cosf(pitch),2)*powf(cosf(roll),2)+y_inertia*powf(cosf(pitch),2)*powf(sinf(roll),2)+x_inertia*powf(sinf(pitch),2));

    float roll_vel = prev_states[10] + roll_acc *time_step;
    float pitch_vel = prev_states[11] + pitch_acc *time_step;
    float yaw_vel = prev_states[12] + yaw_acc *time_step;

    float x_acc = (1/mass)*(sinf(yaw)*sinf(roll)-cosf(yaw)*cosf(roll)*sinf(pitch))*(thrust_constant*ang_velo_m1 + thrust_constant*ang_velo_m2+thrust_constant*ang_velo_m3+thrust_constant*ang_velo_m4);
    float y_acc = (1/mass)*(cosf(yaw)*sinf(roll)+cosf(roll)*sinf(pitch)*sinf(yaw))*(thrust_constant*ang_velo_m1 + thrust_constant*ang_velo_m2 + thrust_constant*ang_velo_m3 + thrust_constant*ang_velo_m4);
    float z_acc = (1/mass)*(cosf(pitch)*cosf(roll))*(thrust_constant*ang_velo_m1 + thrust_constant*ang_velo_m2 + thrust_constant*ang_velo_m3 + thrust_constant*ang_velo_m4)-gravity;

    float x_vel = prev_states[7] + x_acc * time_step;  
    float y_vel = prev_states[8] + y_acc * time_step;  
    float z_vel = prev_states[9] + z_acc * time_step;  
    */
    float roll_acc, pitch_acc, yaw_acc, roll_vel, pitch_vel, yaw_vel, x_acc, y_acc, z_acc, x_vel, y_vel, z_vel;

    float dynamics_vector[num_of_states] = {x_acc,y_acc,z_acc,roll_acc,pitch_acc,yaw_acc,x_vel,y_vel,z_vel,roll_vel,pitch_vel,yaw_vel};
    float output_vector[num_of_states] ={}; 
    float input[num_of_inputs] = {ang_velo_m1, ang_velo_m2, ang_velo_m3, ang_velo_m4};
    float state_estimate[num_of_states];
    float state_predict[num_of_states];
    float output[num_of_states];
    float output_est[num_of_states];

    float identity[num_of_states][num_of_states]; //Rember at denne skal laves til en identitets matrix på et tidspunkt
    float est_error_cov[num_of_states][num_of_states];
    float predict_error_cov[num_of_states][num_of_states]; //Denne skal starte med værdier da den er initial estiamte eller noget til at starte med
    float dynamics_lin_matrix[num_of_states][num_of_states];
    float q[num_of_states][num_of_states];
    float output_lin_matrix[num_of_states][num_of_states];
    float r[num_of_states][num_of_states];


    void calculateKalmanGain()
    {

    }

   
    void updateStateEstimate()
    {

    }

    void calculateStatePrediction()
    {

    }

    void calculateErrorCovariance()
    {

    }

    void calculateNextErrorCovariance()
    {

    }

    void dynamicsFunction(VectorN<float,num_of_states>x, VectorN<float,num_of_states> u)
    {

    }
};
