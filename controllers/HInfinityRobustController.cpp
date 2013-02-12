#include "HInfinityRobustController.h"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;


namespace DQ_robotics
{



HInfinityRobustController::HInfinityRobustController(DQ_kinematics robot, MatrixXd feedback_gain) : DQ_controller()
{

    //Initialization of argument parameters
    robot_dofs_     = (robot.links() - robot.n_dummy());
    robot_          = robot;
    kp_             = feedback_gain;


    //Initilization of remaining parameters
    thetas_         = MatrixXd(robot_dofs_,1);
    delta_thetas_   = MatrixXd::Zero(robot_dofs_,1);

    reference_state_variables_ = MatrixXd(8,1);
    measured_state_variables_  = MatrixXd(8,1);

    N_                 = MatrixXd(8,robot_dofs_);
    task_jacobian_     = MatrixXd(8,robot_dofs_);
    N_pseudoinverse_   = MatrixXd(robot_dofs_,8);

    error_             = MatrixXd(8,1);

    C8_ = C8(); 

    end_effector_pose_ = DQ(0,0,0,0,0,0,0,0);

}

VectorXd HInfinityRobustController::getNewJointPositions(DQ reference, VectorXd thetas)
{

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
    error_ = Hminus8(reference)*(C8())*(reference_state_variables_ - measured_state_variables_);

    N_ = Hminus8(reference)*(C8())*task_jacobian_;

    N_pseudoinverse_ = pseudoInverse(N_);
               
    delta_thetas_ = N_pseudoinverse_*kp_*error_;

    // Send updated thetas to simulation
    return (thetas_ + delta_thetas_);

}

VectorXd HInfinityRobustController::getNewJointVelocities(DQ reference, VectorXd thetas)
{

    std::cout << std::endl << "getNewJointVelocities() not implemented yet" << std::endl;
    return VectorXd(1,1);

}





}
