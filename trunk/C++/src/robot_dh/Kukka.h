/**
Kukka Robot DH Parameters

\author Murilo Marques Marinho
\since 02/2013
*/

#ifndef DQ_ROBOTICS_KUKKA_DH_H
#define DQ_ROBOTICS_KUKKA_DH_H

#include"../DQ_kinematics.h"
#include<Eigen/Dense>
#include<cmath>

using namespace Eigen;

namespace DQ_robotics{

    DQ_kinematics KukkaKinematics()
    {
        const double pi2 = M_PI_2;

	    Matrix<double,4,7> kukka_dh(4,7);
	    kukka_dh <<  0,     0,     0,   0,   0,    0,   0,
                     0.310, 0,     0.4, 0,   0.39, 0,   0,
                     0,     0,     0,   0,   0,    0,   0,
                     pi2,   -pi2, -pi2, pi2, pi2, -pi2, 0;
	    DQ_kinematics kukka(kukka_dh,"standard");

        return kukka;        
    };

}

#endif
