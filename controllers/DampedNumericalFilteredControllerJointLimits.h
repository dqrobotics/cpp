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


#ifndef DQDAMPEDNUMERICALFILTEREDCONTROLLERJOINTLIMITS_H
#define DQDAMPEDNUMERICALFILTEREDCONTROLLERJOINTLIMITS_H

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../DQ_controller.h"
#include <Eigen/Dense>

using namespace Eigen;


namespace DQ_robotics
{



class DampedNumericalFilteredControllerJointLimits : public DQ_controller
{

public: //variables

private: //variables

    DQ_kinematics robot_;
    int robot_dofs_;

    MatrixXd kp_;
    double beta_;
    double lambda_max_;
    double epsilon_;

    //Joint Limit Related
    VectorXd upper_joint_limits_;
    VectorXd lower_joint_limits_;
    VectorXd original_dummy_joints_;

    VectorXd thetas_;
    VectorXd delta_thetas_;

    VectorXd error_;

    MatrixXd task_jacobian_;
    MatrixXd task_jacobian_pseudoinverse_;

    JacobiSVD<MatrixXd> svd_;
    VectorXd singular_values_;
    MatrixXd svd_sigma_inverted_;
    MatrixXd identity_;

    DQ end_effector_pose_;

    DQ dq_one_;

public: //methods

    DampedNumericalFilteredControllerJointLimits(DQ_kinematics robot, VectorXd upper_joint_limits, VectorXd lower_joint_limits, 
                                                           MatrixXd feedback_gain, double beta, double lambda_max, double epsilon);
    ~DampedNumericalFilteredControllerJointLimits(){};

    VectorXd getNewJointPositions( const DQ reference, const VectorXd thetas);
    VectorXd getNewJointVelocities( const DQ reference, const VectorXd thetas);

private: //methods

};



}


#endif
