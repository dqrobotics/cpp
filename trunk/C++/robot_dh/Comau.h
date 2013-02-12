/**
Comau Robot DH Parameters

\author Murilo Marques Marinho
\since 02/2013
*/

#ifndef DQ_ROBOTICS_COMAU_DH_H
#define DQ_ROBOTICS_COMAU_DH_H

#include"../DQ_kinematics.h"
#include<Eigen/Dense>
#include<cmath>

using namespace Eigen;

namespace DQ_robotics{

    DQ_kinematics ComauKinematics()
    {
        const double pi2 = M_PI_2;
        const double pi  = M_PI;

	    Matrix<double,5,7> comau_dh(5,7);
	    comau_dh << 0,   -pi2,   pi2,    0,       0,    0,     pi,
                   -0.45, 0,     0,     -0.64707, 0,   -0.095, 0,
                    0,    0.150, 0.590,  0.13,    0,    0,     0,
                    pi,   pi2,   pi,    -pi2,    -pi2,  pi2,   pi,
                    0,    0,     0,      0,       0,    0,     1;

	    DQ_kinematics comau(comau_dh,"standard");

        return comau;        
    };

}

#endif
