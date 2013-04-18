/**
WAM Robot DH Parameters

*/

#ifndef DQ_ROBOTICS_WAM4_DH_H
#define DQ_ROBOTICS_WAM4_DH_H

#include"../DQ_kinematics.h"
#include<Eigen/Dense>
#include<cmath>

using namespace Eigen;

namespace DQ_robotics{

    DQ_kinematics Wam4Kinematics()
    {
        const double pi2 = M_PI_2;

	    Matrix<double,4,4> wam4_dh(4,4);
	    wam4_dh << 0,    0,    0,      0,      
                  0,    0,    0.55,   0,
                  0,    0,    0.045, -0.045,
                  -pi2, pi2, -pi2,    pi2;

	    DQ_kinematics wam4(wam4_dh,"standard");

        return wam4;        
    };

}

#endif
