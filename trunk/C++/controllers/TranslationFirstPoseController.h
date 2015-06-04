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


#ifndef DQTRANSLATIONFIRSTPOSECONTROLLER_H
#define DQTRANSLATIONFIRSTPOSECONTROLLER_H

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../DQ_controller.h"
#include <Eigen/Dense>

using namespace Eigen;


namespace DQ_robotics
{



class TranslationFirstPoseController : public DQ_controller
{

public: //variables

private: //variables

    DQ_kinematics robot_;
    int robot_dofs_;

	double rotation_damping_;
	double translation_damping_;

    MatrixXd kp_;
	MatrixXd kr_;

    VectorXd thetas_;
    VectorXd delta_thetas_;

    VectorXd error_translation_;
	VectorXd error_rotation_;

    MatrixXd analytical_jacobian_;
	MatrixXd translation_jacobian_;
	MatrixXd rotation_jacobian_;
    MatrixXd translation_jacobian_pseudoinverse_;
	MatrixXd rotation_jacobian_pseudoinverse_;

	MatrixXd nullspace_projector_;

	MatrixXd identity4_;
	MatrixXd identityDOFS_;

    DQ end_effector_pose_;
	DQ reference_translation_;
	DQ reference_rotation_;


public: //methods
    TranslationFirstPoseController(DQ_kinematics robot, MatrixXd translation_feedback_gain, MatrixXd rotation_feedback_gain, double translation_damping, double rotation_damping);
    ~TranslationFirstPoseController(){};

    VectorXd getNewJointPositions( const DQ reference, const VectorXd thetas);
    VectorXd getNewJointVelocities( const DQ reference, const VectorXd thetas);

private: //methods

};



}


#endif
