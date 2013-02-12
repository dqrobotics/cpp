/**
*  All controllers should inherit this class.
*/


#ifndef DQCONTROLLER_H
#define DQCONTROLLER_H

#include "DQ.h"
#include <Eigen/Dense>

using namespace Eigen;


namespace DQ_robotics
{



class DQ_controller
{

public: //variables

private: //variables

public: //methods
    DQ_controller(){};
    ~DQ_controller(){};

    virtual VectorXd getNewJointPositions(DQ reference, VectorXd thetas)=0;
    virtual VectorXd getNewJointVelocities(DQ reference, VectorXd thetas)=0;

private: //methods



};



}


#endif
