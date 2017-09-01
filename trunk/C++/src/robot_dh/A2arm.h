/**
Schunk Robot DH Parameters

\author Murilo Marques Marinho
\since 09/2013
*/

#ifndef DQ_ROBOTICS_A2ARM_DH_H
#define DQ_ROBOTICS_A2ARM_DH_H

#include"../DQ_kinematics.h"
#include<Eigen/Dense>
#include<cmath>

using namespace Eigen;

namespace DQ_robotics{

    DQ_kinematics A2armKinematics()
    {
        const double pi2 = M_PI_2;
        const double pi  = M_PI;
	    Matrix<double,4,7> a2arm_dh(4,7);
	    a2arm_dh <<  pi2,        pi2,      -pi2,     pi,   pi,     -pi2,  pi,
	                 0.18465,    0,         0.27857, 0,    0.27747, 0,    0,
	                 0,         -0.03175,  -0.00502, 0,    0,       0,    0.04414,
	                -pi2,       -pi2,       pi2,     pi2,  pi2,    -pi2,  pi;
	    DQ_kinematics a2arm(a2arm_dh,"standard");

        return a2arm;        
    };

}

#endif
