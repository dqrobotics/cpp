#include "DampedNumericalFilteredController.h"

namespace DQ_robotics
{



DampedNumericalFilteredController::DampedNumericalFilteredController(DQ_kinematics robot, MatrixXd kp, double beta, double lambda_max, double epsilon) : DQ_controller()
{
    //Constants
    dq_one_ = DQ(1);

    //Initialization of argument parameters
    robot_dofs_     = (robot.links() - robot.n_dummy());
    robot_          = robot;
    kp_             = kp;
    ki_             = MatrixXd::Zero(kp.rows(),kp.cols());
    kd_             = MatrixXd::Zero(kp.rows(),kp.cols());
    beta_           = beta;
    lambda_max_     = lambda_max;

    //Initilization of remaining parameters
    thetas_         = MatrixXd(robot_dofs_,1);
    delta_thetas_   = MatrixXd::Zero(robot_dofs_,1);

    task_jacobian_      = MatrixXd(8,robot_dofs_);
    svd_                = JacobiSVD<MatrixXd>(robot_dofs_,8);
    svd_sigma_inverted_ = MatrixXd::Zero(robot_dofs_,8);
    identity_           = Matrix<double,8,8>::Identity();

    error_              = MatrixXd::Zero(8,1);
    integral_error_     = MatrixXd::Zero(8,1);
    last_error_         = MatrixXd::Zero(8,1);
    at_least_one_error_ = false;

    end_effector_pose_ = DQ(0,0,0,0,0,0,0,0);

}


DampedNumericalFilteredController::DampedNumericalFilteredController( const DQ_kinematics& robot, const MatrixXd& kp, const MatrixXd& ki, const MatrixXd& kd, const double& beta, const double& lambda_max, const double& epsilon) : DQ_controller()
{
    //Constants
    dq_one_ = DQ(1);

    //Initialization of argument parameters
    robot_dofs_     = (robot.links() - robot.n_dummy());
    robot_          = robot;
    kp_             = kp;
    ki_             = ki;
    kd_             = kd;
    beta_           = beta;
    lambda_max_     = lambda_max;

    //Initilization of remaining parameters
    thetas_         = MatrixXd(robot_dofs_,1);
    delta_thetas_   = MatrixXd::Zero(robot_dofs_,1);

    task_jacobian_      = MatrixXd(8,robot_dofs_);
    svd_                = JacobiSVD<MatrixXd>(robot_dofs_,8);
    svd_sigma_inverted_ = MatrixXd::Zero(robot_dofs_,8);
    identity_           = Matrix<double,8,8>::Identity();

    error_              = MatrixXd::Zero(8,1);
    integral_error_     = MatrixXd::Zero(8,1);
    last_error_         = MatrixXd::Zero(8,1);
    at_least_one_error_ = false;


    end_effector_pose_ = DQ(0,0,0,0,0,0,0,0);


}


VectorXd DampedNumericalFilteredController::getNewJointPositions( const DQ reference, const VectorXd thetas)
{

    delta_thetas_ = getNewJointVelocities(reference, thetas);

    // Send updated thetas to simulation
    return (thetas_ + delta_thetas_);

}

VectorXd DampedNumericalFilteredController::getNewJointVelocities( const DQ reference, const VectorXd thetas)
{
    thetas_ = thetas;
    ///--Controller Step
           
    //Calculate jacobian
    task_jacobian_  = robot_.analyticalJacobian(thetas_);
    
    // Recalculation of measured data.
    // End effectors pose
    end_effector_pose_ = robot_.fkm(thetas_);

    //Error
    last_error_      = error_;
    error_           = vec8(reference - end_effector_pose_);
    integral_error_ += error_;
    //error_ = vec8(dq_one_ - conj(end_effector_pose_)*reference);

    svd_.compute(task_jacobian_, ComputeFullU);
    singular_values_ = svd_.singularValues();

    //Damping Calculation
    double sigma_min = singular_values_(5);
    VectorXd u_min = svd_.matrixU().col(5);
    double lambda = lambda_max_;
    if (sigma_min < epsilon_)
    {
        lambda = (1-(sigma_min/epsilon_)*(sigma_min/epsilon_))*lambda_max_*lambda_max_;
    }

    //We want to solve the equation J+ = J^T(JJ^T+aI)^-1, in which the matrix 
    //being inverted is obviously positive definite if a > 0.
    //The solver gives us the solution to X = A^-1.B
    //Therefore I chose to find X^T = B^T(A^T)^-1 then take the transpose of X.
    task_jacobian_pseudoinverse_ = ((task_jacobian_*task_jacobian_.transpose() + (beta_*beta_)*identity_ + (lambda*lambda)*u_min*u_min.transpose()).transpose()).ldlt().solve(task_jacobian_);
    task_jacobian_pseudoinverse_.transposeInPlace();

    if( at_least_one_error_ )
      delta_thetas_ = task_jacobian_pseudoinverse_*( kp_*error_ + ki_*integral_error_ + kd_*(error_ - last_error_) );
    else
    {
      at_least_one_error_ = true;
      delta_thetas_ = task_jacobian_pseudoinverse_*( kp_*error_ + ki_*integral_error_ );
    }

    return delta_thetas_;

}

void DampedNumericalFilteredController::setPGain( const MatrixXd new_kp )
{
    kp_ = new_kp;
}

void DampedNumericalFilteredController::setIGain( const MatrixXd new_ki )
{
    ki_ = new_ki;
}



}
