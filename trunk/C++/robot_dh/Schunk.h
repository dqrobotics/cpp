/**
Schunk Robot DH Parameters

\author Murilo Marques Marinho
\since 02/2013
*/

#ifndef DQ_ROBOTICS_SCHUNK_DH_H
#define DQ_ROBOTICS_SCHUNK_DH_H

#include"../DQ_kinematics.h"
#include<Eigen/Dense>
#include<cmath>

using namespace Eigen;

namespace DQ_robotics{

    DQ_kinematics SchunkKinematics()
    {
      const double pi2 = M_PI_2;
	    Matrix<double,4,7> schunk_dh(4,7);
	    schunk_dh << 0,     0,   0,     0,   0,      0,  0,
	                 0.3,   0,   0.328, 0,   0.2765, 0,  0.1793,
	                 0,     0,   0,     0,   0,      0,  0,
	                -pi2,   pi2,-pi2,   pi2,-pi2,    pi2,0;
	    DQ_kinematics schunk(schunk_dh,"standard");

      return schunk;        
    };

}

#endif
