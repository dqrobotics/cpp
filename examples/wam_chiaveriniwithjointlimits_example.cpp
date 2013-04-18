/**
Wam Control Example with joint limits consideration in control loop.

\author Murilo Marques Marinho
\since 04/2013

*/

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../DQ_controller.h"
#include "../controllers/DampedNumericalFilteredControllerJointLimits.h"
#include "../robot_dh/WAM.h" //Has SchunkKinematics()

using namespace Eigen;
using namespace DQ_robotics;

int main(void)
{
    //Gain Matrix
	Matrix<double,8,8> kp = Matrix<double,8,8>::Zero(8,8);
	Matrix<double,8,1> kp_diagonal(8,1);
    kp_diagonal << 0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8;
    kp.diagonal() = kp_diagonal;

    //Initial Joint Values
    Matrix<double,7,1> thetas;
    thetas << 0,0,0,0,0,0,0;

    //End Effector Pose eff_pose_reference
    DQ eff_pose_reference(1,0,0,0,0,0,0,0.652495);

 	//Robot DH
	DQ_kinematics wam = WamKinematics();
    //Upper Limits (If using openRave, it would be easier to just call probot->GetDOFLimits() and store the result in vectors of the eigen library)
    Matrix<double,7,1> upper_limits;
    upper_limits <<  2.61799, 1.97222, 2.74017, 3.14159 , 1.309  , 1.5708, 3.00197;
    //Lower Limits
    Matrix<double,7,1> lower_limits;
    lower_limits << -2.61799,-1.97222,-2.74017,-0.872665,-4.79966,-1.5708,-3.00197;

    std::cout << std::endl <<"Chiaverini's Singularity-Robust Controller with Joint Limit Consideration" << std::endl;
    DampedNumericalFilteredControllerJointLimits controller(wam, upper_limits, lower_limits, kp, 0.01, 0.01, 0.001);

    //Control Loop Variables
    DQ eff_pose_current(0);
    DQ eff_pose_difference(20); //An initial large value so it does not break from the control loop
    double control_threshold = 1.e-5;
    int control_step_count=0;

    std::cout << std::endl << "Initial Thetas" << std::endl << thetas << std::endl;
    std::cout << std::endl << "Robot initial FKM = " << wam.fkm(thetas) << std::endl;

    //Control Loop
    while(eff_pose_difference.vec8().norm() > control_threshold)
    {   

        //One controller step
        thetas = controller.getNewJointPositions(eff_pose_reference,thetas);

        //End of control check
        eff_pose_current    = wam.fkm(thetas);
        eff_pose_difference = (eff_pose_current - eff_pose_reference);

        //Count Steps
        control_step_count++;

    }

    std::cout << std::endl <<"Control Loop Ended In " << control_step_count << " Steps" << std::endl;
    std::cout << std::endl <<"End Effector Pose Reference" << std::endl << eff_pose_reference << std::endl;
    std::cout << std::endl <<"End Effector Final Pose" << std::endl << eff_pose_current << std::endl;
    std::cout << std::endl <<"End Effector Final Pose Difference" << std::endl << eff_pose_difference << std::endl;
    std::cout << std::endl <<"Final Thetas" << std::endl << thetas << std::endl;



    return 0;
}




