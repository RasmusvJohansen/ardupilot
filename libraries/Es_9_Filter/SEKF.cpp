#include "SEKF.h"



void SEKF::calculateKalmanGain()
{
    float numerator[num_of_states];
    float denominator[num_of_states];
    for(size_t row{0}; row < num_of_states; row++)
    {
        for(size_t col{0}; col <num_of_states; col++)
        {
            numerator[row] += prev_p[row][col] * h[row][col];
            denominator[row] += h[row][col] * prev_p[row][col] * h[row][col];
        }
        denominator[row] = denominator[row] + r[row][row];
        k[row] = numerator[row]/denominator[row];
    }
}

void SEKF::updateStateEstimate()
{
    for (size_t row{0}; row < num_of_states; row++)
    {
            x[row] = prev_x[row] + k[row] * (y[row] - y_est[row]);
    }
}

void SEKF::calculateStateEstimate()
{
        prev_x = dynamicsFunction(x, u ).operator+(w);
}

VectorN<float,SEKF::num_of_states> SEKF::dynamicsFunction(VectorN<float,num_of_states> x,VectorN<float,num_of_states> u)
{

    

    return x + u;
}

void SEKF::calculateErrorCovariance()
{
    float temp_error_cov[num_of_states][num_of_states];
    for (size_t row{0}; row < num_of_states; row++)
    {
        for (size_t col{0}; col < num_of_states; col++)
        {
            temp_error_cov[row][col] = (identity[row][col] - k[row] * h[row][col]);
            p[row][col] = temp_error_cov[row][col] * prev_p[row][col] * temp_error_cov[col][row] + k[row]*r[row][col]*k[row];
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
                temp_next_error_cov[row][col] += f[row][inner] * p[inner][col];
            }
        }   
    }
    for (size_t row = 0; row < num_of_states; row++)
    {
        for (size_t col = 0; col < num_of_states; col++)
        {
            for (size_t inner = 0; inner < num_of_states; inner++)
            {
                next_p[row][col] += temp_next_error_cov[row][inner] * f[inner][col];
            }
            next_p[row][col] = next_p[row][col] + q[row][col];
        }   
    }
}

