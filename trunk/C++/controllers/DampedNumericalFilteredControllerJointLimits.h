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
