/**
ax18 Robot DH Parameters

\author Murilo Marques Marinho
\since 02/2013
*/

#ifndef DQ_ROBOTICS_AX18_DH_H
#define DQ_ROBOTICS_AX18_DH_H

#include"../DQ_kinematics.h"
#include<Eigen/Dense>
#include<cmath>

using namespace Eigen;

namespace DQ_robotics{

    DQ_kinematics Ax18Kinematics()
    {
        const double pi2 = M_PI_2;

	    Matrix<double,5,6> ax18_dh(5,6);
	    ax18_dh <<  -pi2,  0,   -pi2,  0,     -pi2,  0,
                     0,    159,  0,    22.25,  0,    0,
                     0,    0,   -pi2,  0,     -pi2,  0,
                     167,  0,    0,    81.5,   41,   0,
                     0,    0,    0,    1,      0,    0;
	    DQ_kinematics ax18(ax18_dh,"standard");

        return ax18;        
    };

}

#endif
