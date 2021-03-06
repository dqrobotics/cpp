/**
 ax18 Robot DH Parameters
 
 \author Murilo Marques Marinho
 \since 02/2013
 */

/*
 Modified version
 author Juan José Quiroz
 since 06/2016
 changes:
 -Virtual Joint was removed
 -Robot dimensions are in meters now.
 */

#ifndef DQ_ROBOTICS_AX18_DH_H
#define DQ_ROBOTICS_AX18_DH_H

#include<dqrobotics/legacy/DQ_kinematics.h>
#include<eigen3/Eigen/Dense>
#include<cmath>

using namespace Eigen;

namespace DQ_robotics{
    
    DQ_kinematics Ax18Kinematics()
    {
        const double pi2 = M_PI_2;
        
        Matrix<double,5,5> ax18_dh(5,5);
        ax18_dh <<   0,    0,   -pi2,  -pi2,   -pi2,
        0.167,  0,    0,    0.1225,    0,
        0,    0.159,   0.02225,  0 ,   0,
        -pi2,  0,   -pi2,      -pi2,    0,
        0,    0,    0,    0,           0;
        
        DQ_kinematics ax18(ax18_dh,"standard");
        
        return ax18;
    };
    
}

#endif
