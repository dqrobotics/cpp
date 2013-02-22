#include "DampedNumericalFilteredController.h"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;


namespace DQ_robotics
{



DampedNumericalFilteredController::DampedNumericalFilteredController(DQ_kinematics robot, MatrixXd feedback_gain, double beta, double lambda_max, double epsilon) : DQ_controller()
{
    //Constants
    dq_one_ = DQ(1);

    //Initialization of argument parameters
    robot_dofs_     = (robot.links() - robot.n_dummy());
    robot_          = robot;
    kp_             = feedback_gain;
    beta_           = beta;
    lambda_max_     = lambda_max;

    //Initilization of remaining parameters
    thetas_         = MatrixXd(robot_dofs_,1);
    delta_thetas_   = MatrixXd::Zero(robot_dofs_,1);

    task_jacobian_     = MatrixXd(8,robot_dofs_);
    task_jacobian_pseudoinverse_   = MatrixXd(robot_dofs_,8);
    svd_ = JacobiSVD<MatrixXd>(robot_dofs_,8);
    svd_sigma_inverted_ = MatrixXd::Zero(robot_dofs_,8);
    identity_           = Matrix<double,8,8>::Identity();

    error_             = MatrixXd(8,1);

    end_effector_pose_ = DQ(0,0,0,0,0,0,0,0);

}

VectorXd DampedNumericalFilteredController::getNewJointPositions(DQ reference, VectorXd thetas)
{

    delta_thetas_ = getNewJointVelocities(reference, thetas);

    // Send updated thetas to simulation
    return (thetas_ + delta_thetas_);

}

VectorXd DampedNumericalFilteredController::getNewJointVelocities(DQ reference, VectorXd thetas)
{

    ///--Remapping arguments
    thetas_ = thetas;

    ///--Controller Step
           
    //Calculate jacobian
    task_jacobian_  = robot_.analyticalJacobian(thetas_);
    
    // Recalculation of measured data.
    // End effectors pose
    end_effector_pose_ = robot_.fkm(thetas_);

    //Error
    error_ = vec8(reference - end_effector_pose_);
    //error_ = vec8(dq_one_ - conj(end_effector_pose_)*reference);

    svd_.compute(task_jacobian_, ComputeFullU);
    singular_values_ = svd_.singularValues();

    //Damping Calculation
    double sigma_min = singular_values_(robot_dofs_-1);
    VectorXd u_min = svd_.matrixU().col(robot_dofs_-1);
    double lambda = lambda_max_;
    if (sigma_min < epsilon_)
    {
        lambda = (1-(sigma_min/epsilon_)*(sigma_min/epsilon_))*lambda_max_*lambda_max_;
    }

    task_jacobian_pseudoinverse_ = (task_jacobian_.transpose())*((task_jacobian_*task_jacobian_.transpose() + (beta_*beta_)*identity_ + (lambda*lambda)*u_min*u_min.transpose()).inverse());

    delta_thetas_ = task_jacobian_pseudoinverse_*kp_*error_;

    return delta_thetas_;

}





}
