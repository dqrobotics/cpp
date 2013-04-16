#include "DampedNumericalFilteredControllerJointLimits.h"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;


namespace DQ_robotics
{

DampedNumericalFilteredControllerJointLimits::DampedNumericalFilteredControllerJointLimits(DQ_kinematics robot, VectorXd upper_joint_limits, VectorXd lower_joint_limits, MatrixXd feedback_gain, double beta, double lambda_max, double epsilon) : DQ_controller()
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

    task_jacobian_                 = MatrixXd(8,robot_dofs_);
    task_jacobian_pseudoinverse_   = MatrixXd(robot_dofs_,8);
    svd_                           = JacobiSVD<MatrixXd>(robot_dofs_,8);
    svd_sigma_inverted_            = MatrixXd::Zero(robot_dofs_,8);
    identity_                      = Matrix<double,8,8>::Identity();

    error_             = MatrixXd(8,1);

    end_effector_pose_ = DQ(0,0,0,0,0,0,0,0);
    
    upper_joint_limits_    = upper_joint_limits;
    lower_joint_limits_    = lower_joint_limits;
    original_dummy_joints_ = robot_.dummy(); 
}


VectorXd DampedNumericalFilteredControllerJointLimits::getNewJointPositions(DQ reference, VectorXd thetas)
{

    delta_thetas_ = getNewJointVelocities(reference, thetas);

    // Send updated thetas to simulation
    return (thetas_ + delta_thetas_);

}

VectorXd DampedNumericalFilteredControllerJointLimits::getNewJointVelocities(DQ reference, VectorXd thetas)
{

    thetas_ = thetas;

    VectorXd pseudo_dummy_joint_marker = VectorXd::Zero(robot_dofs_);
    VectorXd pseudo_thetas = thetas_;
    VectorXd pseudo_delta_thetas;
    VectorXd possible_new_thetas;    

    MatrixXd original_dh_matrix = robot_.getDHMatrix();
    MatrixXd step_dh_matrix = original_dh_matrix;
    DQ_kinematics pseudo_robot(original_dh_matrix);

    bool should_break_loop = false;
    while(not should_break_loop){

        //Calculate jacobian
        task_jacobian_  = pseudo_robot.analyticalJacobian(pseudo_thetas);
  
        // Recalculation of measured data.
        end_effector_pose_ = pseudo_robot.fkm(pseudo_thetas);

        //Error
        error_ = vec8(reference - end_effector_pose_);
        //error_ = vec8(dq_one_ - conj(end_effector_pose_)*reference);

        ///Inverse calculation
        svd_.compute(task_jacobian_, ComputeFullU);

        singular_values_ = svd_.singularValues();

        //Damping Calculation
        int current_step_relevant_dof = pseudo_robot.links() - pseudo_robot.n_dummy();
        double sigma_min = singular_values_( current_step_relevant_dof - 1);
        VectorXd u_min = svd_.matrixU().col( current_step_relevant_dof - 1);
        double lambda = lambda_max_;
        if (sigma_min < epsilon_)
        {
            lambda = (1-(sigma_min/epsilon_)*(sigma_min/epsilon_))*lambda_max_*lambda_max_;
        }

        task_jacobian_pseudoinverse_ =    (task_jacobian_.transpose())*((task_jacobian_*task_jacobian_.transpose() 
                                            + (beta_*beta_)*identity_ + (lambda*lambda)*u_min*u_min.transpose()).inverse());

        pseudo_delta_thetas = task_jacobian_pseudoinverse_*kp_*error_;

        //Set delta thetas as if the loop was ending now.
        for(int i = 0, j = 0; i < robot_dofs_; i++)
        {
            if(pseudo_dummy_joint_marker(i) == 1)
                delta_thetas_(i) = 0;
            else{
                delta_thetas_(i) = pseudo_delta_thetas(j);
                j++;
            }
        }

        possible_new_thetas = thetas_ + delta_thetas_;

        //Verify if loop should end
        should_break_loop = true;
        int j=0;
        for(int i = 0; i < robot_dofs_; i++){

            if(pseudo_dummy_joint_marker(i) == 0){

                if(    possible_new_thetas(i) > upper_joint_limits_(i)
                    || possible_new_thetas(i) < lower_joint_limits_(i) )
                {
                    //std::cout << std::endl << "YOU SHALL NOT PASS" << std::endl;
                    pseudo_dummy_joint_marker(i) = 1;
                    should_break_loop = false;
                    step_dh_matrix(4,i) = 1; //Set matrix as dummy.
                    step_dh_matrix(0,i) = thetas_(i); //Set matrix theta as a fixed value.
       
                }
                else{
                    pseudo_thetas(j) = thetas_(i);
                    j++;      
                }
            }

        }
        if(j==0){
            //std::cout << std::endl << "Robot will be unable to get out of this configuration using this controller." << std::endl;
            delta_thetas_ = VectorXd::Zero(robot_dofs_);
            break;       
        }

        pseudo_thetas.resize(j);

        //Change DH
        pseudo_robot = DQ_kinematics(step_dh_matrix);

    }

/*

    ///--Remapping arguments
    thetas_ = thetas;

    //Non-class variables used in control loop.
    VectorXd possible_new_thetas;
    VectorXd pseudo_dummy_joints = original_dummy_joints_;
    VectorXd pseudo_delta_thetas;
    VectorXd pseudo_thetas = thetas_;
    bool limits_were_violated = true;

    int current_step_relevant_dof;

    //Verify if joint limits where violated
    while (limits_were_violated)
    {
        current_step_relevant_dof = ( robot_.links() - robot_.n_dummy() );
        limits_were_violated = false;

        //Calculate jacobian
        task_jacobian_  = robot_.analyticalJacobian(pseudo_thetas);
        
        // Recalculation of measured data.
        end_effector_pose_ = robot_.fkm(pseudo_thetas);

        //Error
        error_ = vec8(reference - end_effector_pose_);
        //error_ = vec8(dq_one_ - conj(end_effector_pose_)*reference);

        ///Inverse calculation
        svd_.compute(task_jacobian_, ComputeFullU);

        singular_values_ = svd_.singularValues();

        //Damping Calculation
        sigma_min = singular_values_( current_step_relevant_dof -1);
        u_min = svd_.matrixU().col( current_step_relevant_dof -1);
        lambda = lambda_max_;
        if (sigma_min < epsilon_)
        {
            lambda = (1-(sigma_min/epsilon_)*(sigma_min/epsilon_))*lambda_max_*lambda_max_;
        }

        task_jacobian_pseudoinverse_ =    (task_jacobian_.transpose())*((task_jacobian_*task_jacobian_.transpose() 
                                        + (beta_*beta_)*identity_ + (lambda*lambda)*u_min*u_min.transpose()).inverse());

        pseudo_delta_thetas = task_jacobian_pseudoinverse_*kp_*error_;
        pseudo_thetas.resize(current_step_relevant_dof);

        //As the jacobian sizes might have changed, we have to remap the thetas to the right joint indexes.
        for(int i = 0, k = 0, j = 0; i < robot_.links(); i++)
        {
            if(original_dummy_joints_(i) == 0)
                {
                if( pseudo_dummy_joints(i) == 1 )
                {
                    delta_thetas_(k) = 0; //This joint is not supposed to move.
                    k++;   
                }
                else
                {
                    pseudo_thetas(j) = thetas_(k);
                    delta_thetas_(k) = pseudo_delta_thetas(j);
                    j++;
                    k++;
                }
            }
        }

        //std::cout << std::endl << "Milestone 2" << std::endl;

        //Checks all joints and set as dummies those that try going through the joint limits.
        possible_new_thetas = thetas_ + delta_thetas_;

        //Iterate through all links
        for( int i = 0, j = 0; i < robot_.links(); i++)
        {
            //Verify if joint is not already dummy
            if(pseudo_dummy_joints(i) == 0)
            {
                //Verify limits
                if(    possible_new_thetas(j) > upper_joint_limits_(j)
                    || possible_new_thetas(j) < lower_joint_limits_(j) )
                {
                    //If violates joint limits, we should ignore it in jacobian calculations.
                    pseudo_dummy_joints(i) = 1;
                    limits_were_violated = true;
                    std::cout << std::endl << "Joint limit was violated!!" << std::endl;
                }
                j++;
            }
            
        }     

        robot_.setDummy( pseudo_dummy_joints );
        pseudo_thetas.resize( ( robot_.links() - robot_.n_dummy() ) );
        //As the jacobian sizes might have changed, we have to remap the thetas to the right joint indexes.
        for(int i = 0, j = 0; i < robot_dofs_; i++)
        {
            if( pseudo_dummy_joints(i) == 0 )
            {
                pseudo_thetas(j) = thetas_(i);
                j++;
            }
        }



    }

    //Reset dummy joint status
    robot_.setDummy( original_dummy_joints_ );
*/
    return delta_thetas_;

}





}
