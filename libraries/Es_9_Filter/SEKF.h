#include "AP_Math/vectorN.h"
#include "AP_Math/matrixN.h"


class SEKF
{
public:
    /* Skriv funktioner som viser værdier */


private:
    /* Skriv variabler, matricer, vektorer og funktioner her */
    static const int num_of_states{12};

    VectorN<float,num_of_states>k;
    VectorN<float,num_of_states>y;
    VectorN<float,num_of_states>y_est;
    VectorN<float,num_of_states>x;
    VectorN<float,num_of_states>prev_x;

    //MatrixN<float,12>p;
    //MatrixN<float,12>h;

    float identity[num_of_states][num_of_states]; //Rember at denne skal laves til en identitets matrix på et tidspunkt
    float p[num_of_states][num_of_states];
    float prev_p[num_of_states][num_of_states]; //Denne skal starte med værdier da den er initial estiamte eller noget til at starte med
    float h[num_of_states][num_of_states];
    float r[num_of_states][num_of_states];


    void calculateKalmanGain()
    {

    }

    void calculateStateEstimate()
    {

    }

    void calculateErrorCovariance()
    {

    }


};
