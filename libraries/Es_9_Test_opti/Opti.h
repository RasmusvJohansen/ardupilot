


#define ALLOW_DOUBLE_MATH_FUNCTIONS
#define OPTIM_ENABLE_EIGEN_WRAPPERS
#define OPTIM_PI 3.14159265358979
#undef _GLIBCXX_USE_C99_STDIO

#include "optimization/include/optim/optim.hpp"


#include "Eigen/Dense"

class Opti
{
private:


static double ackley_fn(const Eigen::VectorXd& vals_inp, Eigen::VectorXd* grad_out, void* opt_data)
{
    const double x = vals_inp(0);
    const double y = vals_inp(1);

    const double obj_val = 20 + std::exp(1) - 20*std::exp( -0.2*std::sqrt(0.5*(x*x + y*y)) ) - std::exp( 0.5*(std::cos(2 * OPTIM_PI * x) + std::cos(2 * OPTIM_PI * y)) );
            
    return obj_val;
}
        

void doSomething()

{
    v.dot(c);
}

void optim()
{
    Eigen::VectorXd x = 2.0 * Eigen::VectorXd::Ones(2); // initial values: (2,2)
        
    optim::de(x, ackley_fn, nullptr);
    
}





public:
    Eigen::Vector3d v = {1,2,3};   
    Eigen::Vector3d c = {2,4,6};

    
    

};
