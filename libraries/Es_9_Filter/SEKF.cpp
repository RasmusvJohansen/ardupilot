#include "SEKF.h"



void SEKF::calculateKalmanGain()
{
    float numerator[num_of_states];
    float denominator[num_of_states];
    for(int row{0}; row < num_of_states; row++)
    {
        for(int col{0}; col <num_of_states; col++)
        {
            numerator[row] += prev_p[row][col] * h[row][col];
            denominator[row] += h[row][col] * prev_p[row][col] * h[row][col];
        }
        denominator[row] = denominator[row] + r[row][row];
        k[row] = numerator[row]/denominator[row];
    }
}

void SEKF::calculateStateEstimate()
{
    for (int row{0}; row < num_of_states; row++)
    {
            x[row] = prev_x[row] + k[row] * (y[row] - y_est[row]);
    }
}

void SEKF::calculateErrorCovariance()
{
    float temp[num_of_states][num_of_states];
    for (int row{0}; row < num_of_states; row++)
    {
        for (int col{0}; col < num_of_states; col++)
        {
            temp[row][col] = (identity[row][col] - k[row] * h[row][col]);
            p[row][col] = temp[row][col] * prev_p[row][col] * temp[col][row] + k[row]*r[row][col]*k[row];
        }
    }
}
