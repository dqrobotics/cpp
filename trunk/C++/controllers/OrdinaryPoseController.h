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


#ifndef DQORDINARYPOSECONTROLLER_H
#define DQORDINARYPOSECONTROLLER_H

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../DQ_controller.h"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;


namespace DQ_robotics
{



class OrdinaryPoseController : public DQ_controller
{

public: //variables

private: //variables

    DQ_kinematics robot_;
    int robot_dofs_;

    MatrixXd kp_;
    double lambda_;

    VectorXd thetas_;
    VectorXd delta_thetas_;

    VectorXd error_;

    MatrixXd task_jacobian_;
    MatrixXd task_jacobian_pseudoinverse_;

    MatrixXd identity_;

    DQ end_effector_pose_;

    DQ dq_one_;


public: //methods
    OrdinaryPoseController(DQ_kinematics robot, MatrixXd feedback_gain, double lambda);
    ~OrdinaryPoseController(){};

    VectorXd getNewJointPositions( const DQ reference, const VectorXd thetas);
    VectorXd getNewJointVelocities( const DQ reference, const VectorXd thetas);

private: //methods

};



}


#endif
