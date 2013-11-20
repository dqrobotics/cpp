/**
* All controllers should inherit this class.
*
* \author Murilo Marques Marinho (murilomarinho@lara.unb.br)
* \since 2012/07
***********************************************************
*              REVISION HISTORY
***********************************************************
* YYYY/MM/DD   Author (e-mail address)
*            - Description
***********************************************************
* 2013/11/22   Murilo Marques Marinho (murilomarinho@lara.unb.br)
             - Added const qualifiers and references whenever
               possible.
**/

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

    virtual VectorXd getNewJointPositions( const DQ reference, const VectorXd thetas)=0;
    virtual VectorXd getNewJointVelocities( const DQ reference, const VectorXd thetas)=0;

private: //methods



};



}


#endif
