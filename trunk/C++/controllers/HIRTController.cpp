#include "HIRTController.h"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;


namespace DQ_robotics
{



HIRTController::HIRTController( const DQ_kinematics& robot, const Matrix<double,8,1>& B, const double& gamma, const double& alpha ) : DQ_controller()
{

    //Initialization of argument parameters
    robot_dofs_     = (robot.links() - robot.n_dummy());
    robot_          = robot;
    kp_             = MatrixXd::Zero(8,8);
    B_              = B;
    Bw_             = Matrix<double,8,1>::Zero();
    gamma_          = gamma;
    alpha_          = alpha;

    //Initilization of remaining parameters
    thetas_         = MatrixXd(robot_dofs_,1);
    delta_thetas_   = MatrixXd::Zero(robot_dofs_,1);

    old_reference_                  = DQ(0.0);
    runonce_                        = false;
    reference_state_variables_      = MatrixXd(8,1);
    measured_state_variables_       = MatrixXd(8,1);

    N_                 = MatrixXd(8,robot_dofs_);
    task_jacobian_     = MatrixXd(8,robot_dofs_);
    N_pseudoinverse_   = MatrixXd(robot_dofs_,8);

    error_             = MatrixXd(8,1);

    C8_        = C8(); 
    identity8_ = MatrixXd::Identity(8,8);

    end_effector_pose_ = DQ(0,0,0,0,0,0,0,0);

}

VectorXd HIRTController::getNewJointPositions( const DQ reference, const VectorXd thetas)
{

    delta_thetas_ = getNewJointVelocities( reference, thetas);

    // Send updated thetas to simulation
    return (thetas_ + delta_thetas_);

}

VectorXd HIRTController::getNewJointVelocities( const DQ reference, const VectorXd thetas)
{

    if(not runonce)
    {
      old_reference = reference;
      runonce       = true;
    }


    ///--Remapping arguments
    thetas_ = thetas;
    reference_state_variables_ = reference.vec8();

    ///--Controller Step
           
    //Calculate jacobian
    task_jacobian_  = robot_.jacobian(thetas_);
    
    // Recalculation of measured data.
    // End effectors pose
    end_effector_pose_ = robot_.fkm(thetas_);
    measured_state_variables_ = vec8(end_effector_pose_);

    //Error
    error_ = Hminus8(reference)*(C8_)*(reference_state_variables_ - measured_state_variables_);

    N_ = Hminus8(reference)*(C8_)*task_jacobian_;

    N_pseudoinverse_ = pseudoInverse(N_);

    //Recalculation of K (if reference changed)
    if(old_reference_ != reference)
    {
      Bw_ = Hminus8(reference)*(C8_)*B_;
      //std::cout << std::endl << (Bw_.transpose()*Bw_*sqrt(2.0)) << std::endl;
      double bwtbwsqrt2 = (Bw_.transpose()*Bw_*sqrt(2.0)).coeff(0);
      kp_ = (1.0/gamma_) * ( Bw_*Bw_.transpose() + (bwtbwsqrt2/4.0)*identity8_ )*(alpha_/sqrt(bwtbwsqrt2));
      std::cout << std::endl << kp_ << std::endl;
    }

    delta_thetas_ = thetas_ + N_pseudo_inverse_* ( (identity8_ - kp_)*error_ - Hminus8(reference - old_reference)*C8_*vec8(reference) );

    old_reference_ = reference;
   
    return delta_thetas_;

}





}
