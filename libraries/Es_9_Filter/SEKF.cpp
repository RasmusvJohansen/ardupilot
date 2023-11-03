#include "SEKF.h"



void SEKF::calculateKalmanGain()
{
    float numerator[num_of_states];
    float denominator[num_of_states];
    for(size_t row{0}; row < num_of_states; row++)
    {
        for(size_t col{0}; col <num_of_states; col++)
        {
            numerator[row] += predict_error_cov[row][col] * output_lin_matrix[row][col];
            denominator[row] += output_lin_matrix[row][col] * predict_error_cov[row][col] * output_lin_matrix[row][col];
        }
        denominator[row] = denominator[row] + r[row][row];
        k[row] = numerator[row]/denominator[row];
    }
}

void SEKF::updateStateEstimate()
{
    for (size_t row{0}; row < num_of_states; row++)
    {
            state_estimate[row] = state_predict[row] + k[row] * (output[row] - output_est[row]);
    }
}

void SEKF::calculateStatePrediction()
{
    dynamicsFunction(state_estimate, input);
    for (size_t row = 0; row < num_of_states; row++)
    {
        state_predict[row] = dynamics_vector[row] + w[row];
    }
}

void SEKF::dynamicsFunction(VectorN<float,num_of_states> states,VectorN<float,num_of_states> input)
{
    roll_acc = -(cosf(roll)*sinf(roll)*(y_inertia-z_inertia)*powf(pitch_vel,2)+(powf(cosf(roll),2)-powf(sinf(roll),2))*(y_inertia-z_inertia)*pitch_vel*yaw_vel*cosf(yaw)-cosf(roll)*sinf(roll)*(y_inertia-z_inertia)*powf(yaw_acc,2)*powf(cosf(pitch),2)+x_inertia*roll_acc*cosf(roll)*yaw_vel-torque_roll+x_inertia*yaw_acc*sinf(roll))/x_inertia;
    pitch_acc = (torque_pitch+(yaw_vel*(roll_vel*cosf(pitch)*(powf(cosf(roll),2)-powf(sinf(roll),2))-pitch_vel*cosf(roll)*sinf(pitch)*sinf(roll))+yaw_acc*cosf(pitch)*cosf(roll)*sinf(roll))*(y_inertia-z_inertia)+x_inertia*yaw_vel*roll_vel*cosf(pitch)-powf(yaw_vel,2)*cosf(pitch)*sinf(pitch)*(z_inertia*powf(cosf(roll),2)+y_inertia*powf(sinf(roll),2)-x_inertia)+2*y_inertia*pitch_vel*roll_vel*cosf(roll)*sinf(roll)-2*z_inertia*pitch_vel*roll_vel*cosf(roll)*sinf(roll)+pitch_vel*yaw_vel*cosf(roll)*sinf(pitch)*(y_inertia-z_inertia))/(y_inertia*powf(cosf(roll),2)+z_inertia*powf(sinf(roll),2));
    yaw_acc = (torque_yaw + yaw_vel*(2*z_inertia*pitch_vel*cosf(pitch)*powf(cosf(roll),2)*sinf(pitch)-2*x_inertia*pitch_vel*cosf(pitch)*sinf(pitch)-2*y_inertia*roll_vel*powf(cosf(pitch),2)*cosf(roll)*sinf(roll)+2*z_inertia*roll_vel*powf(cosf(pitch),2)*cosf(roll)*sinf(roll)+2*y_inertia*pitch_vel*cosf(pitch)*sinf(pitch)*powf(sinf(pitch),2))-x_inertia*roll_acc*sinf(pitch)-x_inertia*pitch_vel*roll_vel*cosf(pitch)-powf(pitch_vel,2)*cosf(roll)*sinf(pitch)*sinf(roll)*(y_inertia-z_inertia)+pitch_vel*roll_vel*cosf(pitch)*powf(cosf(roll),2)*(y_inertia-z_inertia)-pitch_vel*roll_vel*cosf(pitch)*powf(sinf(roll),2)+pitch_acc*cosf(roll)*cosf(roll)*sinf(roll)*(y_inertia-z_inertia))/(z_inertia*powf(cosf(pitch),2)*powf(cosf(roll),2)+y_inertia*powf(cosf(pitch),2)*powf(sinf(roll),2)+x_inertia*powf(sinf(pitch),2));

    roll_vel = state_predict[10] + roll_acc * time_step;
    pitch_vel = state_predict[11] + pitch_acc * time_step;
    yaw_vel = state_predict[12] + yaw_acc * time_step;

    x_acc = (1/mass)*(sinf(yaw)*sinf(roll)-cosf(yaw) * cosf(roll)*sinf(pitch))*(thrust_constant*ang_velo_m1+thrust_constant*ang_velo_m2+thrust_constant*ang_velo_m3+thrust_constant*ang_velo_m4);
    y_acc = (1/mass)*(cosf(yaw)*sinf(roll)+cosf(roll) * sinf(pitch)*sinf(yaw))*(thrust_constant*ang_velo_m1 + thrust_constant*ang_velo_m2 + thrust_constant*ang_velo_m3 + thrust_constant*ang_velo_m4);
    z_acc = (1/mass)*(cosf(pitch)*cosf(roll))*(thrust_constant*ang_velo_m1 + thrust_constant*ang_velo_m2 + thrust_constant*ang_velo_m3 + thrust_constant*ang_velo_m4)-gravity;

    x_vel = state_predict[7] + x_acc * time_step;  
    y_vel = state_predict[8] + y_acc * time_step;  
    z_vel = state_predict[9] + z_acc * time_step;  
}

void SEKF::calculateErrorCovariance()
{
    float temp_error_cov[num_of_states][num_of_states];
    for (size_t row{0}; row < num_of_states; row++)
    {
        for (size_t col{0}; col < num_of_states; col++)
        {
            temp_error_cov[row][col] = (identity[row][col] - k[row] * output_lin_matrix[row][col]);
            est_error_cov[row][col] = temp_error_cov[row][col] * predict_error_cov[row][col] * temp_error_cov[col][row] + k[row]*r[row][col]*k[row];
        }
    }
}

void SEKF::calculateNextErrorCovariance()
{
    float temp_next_error_cov[num_of_states][num_of_states];
    for (size_t row = 0; row < num_of_states; row++)
    {
        for (size_t col = 0; col < num_of_states; col++)
        {
            for (size_t inner = 0; inner < num_of_states; inner++)
            {
                temp_next_error_cov[row][col] += dynamics_lin_matrix[row][inner] * est_error_cov[inner][col];
            }
        }   
    }
    for (size_t row = 0; row < num_of_states; row++)
    {
        for (size_t col = 0; col < num_of_states; col++)
        {
            for (size_t inner = 0; inner < num_of_states; inner++)
            {
                predict_error_cov[row][col] += temp_next_error_cov[row][inner] * dynamics_lin_matrix[inner][col];
            }
            predict_error_cov[row][col] = predict_error_cov[row][col] + q[row][col];
        }   
    }
}

