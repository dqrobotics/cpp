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


#include "TranslationFirstPoseController.h"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;


namespace DQ_robotics
{



TranslationFirstPoseController::TranslationFirstPoseController(DQ_kinematics robot, MatrixXd translation_feedback_gain, MatrixXd rotation_feedback_gain, double translation_damping, double rotation_damping) : DQ_controller()
{

    //Initialization of argument parameters
    robot_dofs_          = (robot.links() - robot.n_dummy());
    robot_               = robot;
    kp_                  = translation_feedback_gain;
	kr_                  = rotation_feedback_gain;
	translation_damping_ = translation_damping;
	rotation_damping_    = rotation_damping;

    //Initilization of remaining parameters
    thetas_         = MatrixXd(robot_dofs_,1);
    delta_thetas_   = MatrixXd::Zero(robot_dofs_,1);

    analytical_jacobian_  = MatrixXd(8,robot_dofs_);
	rotation_jacobian_    = MatrixXd(4,robot_dofs_);
	translation_jacobian_ = MatrixXd(4,robot_dofs_);

	nullspace_projector_  = MatrixXd(robot_dofs_, robot_dofs_);

    translation_jacobian_pseudoinverse_   = MatrixXd(robot_dofs_,4);
	rotation_jacobian_pseudoinverse_   = MatrixXd(robot_dofs_,4);

    identity4_           = Matrix<double,4,4>::Identity();
	identityDOFS_        = MatrixXd::Identity(robot_dofs_,robot_dofs_);

    error_translation_  = MatrixXd(4,1);
	error_rotation_     = MatrixXd(4,1);

    end_effector_pose_           = DQ(0,0,0,0,0,0,0,0);
	reference_translation_       = DQ(0,0,0,0,0,0,0,0);
	reference_rotation_          = DQ(0,0,0,0,0,0,0,0);

}

VectorXd TranslationFirstPoseController::getNewJointPositions( const DQ reference, const VectorXd thetas)
{

    delta_thetas_ = getNewJointVelocities(reference, thetas);

    // Send updated thetas to simulation
    return (thetas_ + delta_thetas_);

}

VectorXd TranslationFirstPoseController::getNewJointVelocities( const DQ reference, const VectorXd thetas)
{

    ///--Remapping arguments
    thetas_ = thetas;

	//Get translation and rotation individually
	reference_translation_ = reference.translation();
	reference_rotation_    = P(reference);

    ///--Controller Step
           
    //Calculate jacobian and FKM
    analytical_jacobian_  = robot_.analyticalJacobian(thetas_);
    end_effector_pose_    = robot_.fkm(thetas_);

    //Error
    error_translation_ = vec4( reference_translation_ -  end_effector_pose_.translation() );

	error_rotation_    = vec4( reference_rotation_    - P(end_effector_pose_) );

	//Calculate Jacobians
	translation_jacobian_ = translationJacobian(analytical_jacobian_, vec8(end_effector_pose_));
	rotation_jacobian_    = rotationJacobian(analytical_jacobian_);

	//Pseudoinverses calculation
	translation_jacobian_pseudoinverse_ = (translation_jacobian_.transpose())
										 *(translation_jacobian_*translation_jacobian_.transpose() + translation_damping_*translation_damping_*identity4_).inverse();

	rotation_jacobian_pseudoinverse_ =    (rotation_jacobian_.transpose())
										 *(rotation_jacobian_*rotation_jacobian_.transpose() + rotation_damping_*rotation_damping_*identity4_).inverse();
	
	//Nullspace projector
	nullspace_projector_ = (identityDOFS_ - (pseudoInverse(translation_jacobian_))*translation_jacobian_ );

    delta_thetas_ = translation_jacobian_pseudoinverse_ * kp_ * error_translation_ +
					nullspace_projector_ * rotation_jacobian_pseudoinverse_ * kr_ * error_rotation_;


    return delta_thetas_;

}





}
