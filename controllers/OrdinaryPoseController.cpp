/**
* Ordinary Pose Controller in unit dual quaternion space.
*
* \author Murilo Marques Marinho (murilomarinho@lara.unb.br)
* \since 2013/06
*/


#include "OrdinaryPoseController.h"

namespace DQ_robotics
{



OrdinaryPoseController::OrdinaryPoseController(DQ_kinematics robot, MatrixXd feedback_gain, double lambda) : DQ_controller()
{
    //Constants
    dq_one_ = DQ(1);

    //Initialization of argument parameters
    robot_dofs_     = (robot.links() - robot.n_dummy());
    robot_          = robot;
    kp_             = feedback_gain;
    lambda_         = lambda;

    //Initilization of remaining parameters
    thetas_         = MatrixXd(robot_dofs_,1);
    delta_thetas_   = MatrixXd::Zero(robot_dofs_,1);

    task_jacobian_   = MatrixXd(8,robot_dofs_);
    identity_        = Matrix<double,8,8>::Identity();

    error_             = MatrixXd(8,1);

    end_effector_pose_ = DQ(0,0,0,0,0,0,0,0);

}

VectorXd OrdinaryPoseController::getNewJointPositions( const DQ reference, const VectorXd thetas)
{

    delta_thetas_ = getNewJointVelocities(reference, thetas);

    // Send updated thetas to simulation
    return (thetas_ + delta_thetas_);

}

VectorXd OrdinaryPoseController::getNewJointVelocities( const DQ reference, const VectorXd thetas)
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

    //task_jacobian_pseudoinverse_ = (task_jacobian_.transpose())*((task_jacobian_*task_jacobian_.transpose() + (beta_*beta_)*identity_ + (lambda*lambda)*u_min*u_min.transpose()).inverse());

    //We want to solve the equation J+ = J^T(JJ^T+aI)^-1, in which the matrix 
    //being inverted is obviously positive definite if a > 0.
    //The solver gives us the solution to X = A^-1.B
    //Therefore I chose to find X^T = (A^T)^-1B^T then take the transpose of X.
    task_jacobian_pseudoinverse_ = ((task_jacobian_*task_jacobian_.transpose() + (lambda_*lambda_)*identity_ ).transpose()).ldlt().solve(task_jacobian_);
    task_jacobian_pseudoinverse_.transposeInPlace();

    delta_thetas_ = task_jacobian_pseudoinverse_*kp_*error_;

    return delta_thetas_;

}





}
