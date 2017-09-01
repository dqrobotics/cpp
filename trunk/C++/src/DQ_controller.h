/**
(C) Copyright 2015 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
*/

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
