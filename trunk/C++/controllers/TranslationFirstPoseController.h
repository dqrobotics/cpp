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
	Vectorxd error_rotation_;

    MatrixXd analytical_jacobian_;
	MatrixXd translation_jacobian_;
	MatrixXd rotation_jacobian_;
    MatrixXd translation_jacobian_pseudoinverse_;
	MatrixXd rotation_jacobian_pseudoinverse_;

    JacobiSVD<MatrixXd> svd_;
    VectorXd singular_values_;
    MatrixXd svd_sigma_inverted_;
    MatrixXd identity_;

    DQ end_effector_pose_;
	DQ reference_translation_;
	DQ reference_rotation_;

    DQ dq_one_;


public: //methods
    TranslationFirstPoseController(DQ_kinematics robot, MatrixXd translation_feedback_gain, MatrixXd rotation_feedback_gain);
    ~TranslationFirstPoseController(){};

    VectorXd getNewJointPositions(DQ reference, VectorXd thetas);
    VectorXd getNewJointVelocities(DQ reference, VectorXd thetas);

private: //methods

};



}


#endif
