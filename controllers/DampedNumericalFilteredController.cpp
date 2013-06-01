/**
* Chiaverini's Singularity Robust controller for the unit dual quaternion space.
*
* \author Murilo Marques Marinho (murilomarinho@lara.unb.br)
* \since 07/2012
***********************************************************
*              REVISION HISTORY
***********************************************************
* YYYY/MM/DD - Author (e-mail address)
*            - Description
***********************************************************
* 2013/06/01 - Murilo Marques Marinho (murilomarinho@lara.unb.br)
             - Fixed the minimun singular value considered in the 
               numerical filtering. It was considering the last 
               singular value of the matrix when it should be consi-
               dering the 6th.
             - Changed the inversion algorithm in the damped pseudo-
               inverse calculation. It was using .inverse(), which
               is only recomended for matrix.size() <= 4. Changed it
               to use Cholesky decomposition as it is always a posi-
               tive definite matrix.
             - Minor changes in the included files organization.
***********************************************************
*/


#include "DampedNumericalFilteredController.h"

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
    double sigma_min = singular_values_(5);
    VectorXd u_min = svd_.matrixU().col(5);
    double lambda = lambda_max_;
    if (sigma_min < epsilon_)
    {
        lambda = (1-(sigma_min/epsilon_)*(sigma_min/epsilon_))*lambda_max_*lambda_max_;
    }

    //task_jacobian_pseudoinverse_ = (task_jacobian_.transpose())*((task_jacobian_*task_jacobian_.transpose() + (beta_*beta_)*identity_ + (lambda*lambda)*u_min*u_min.transpose()).inverse());

    //We want to solve the equation J+ = J^T(JJ^T+aI)^-1, in which the matrix 
    //being inverted is obviously positive definite if a > 0.
    //The solver gives us the solution to X = A^-1.B
    //Therefore I chose to find X^T = B^T(A^T)^-1 then take the transpose of X.
    task_jacobian_pseudoinverse_ = ((task_jacobian_*task_jacobian_.transpose() + (beta_*beta_)*identity_ + (lambda*lambda)*u_min*u_min.transpose()).transpose()).ldlt().solve(task_jacobian_);
    task_jacobian_pseudoinverse_.transposeInPlace();

    delta_thetas_ = task_jacobian_pseudoinverse_*kp_*error_;

    return delta_thetas_;

}





}
