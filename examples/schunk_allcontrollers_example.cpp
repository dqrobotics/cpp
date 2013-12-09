/**
Schunk DQ Control Examples

\author Murilo Marques Marinho
\since 2013/01
***********************************************************
*              REVISION HISTORY
***********************************************************
* YYYY/MM/DD - Author (e-mail address)
*            - Description
***********************************************************
* 2013/06/01 - Murilo Marques Marinho (murilomarinho@lara.unb.br)
             - Added OrdinaryPoseController
***********************************************************
*/


//DQ Kinematics and Arithmetics
#include "../DQ.h"
#include "../DQ_kinematics.h"
//DQ Controllers
#include "../DQ_controller.h"
#include "../controllers/HInfinityRobustController.h" 
#include "../controllers/HIRTController.h"
#include "../controllers/DampedNumericalFilteredController.h"
#include "../controllers/DampedNumericalFilteredControllerJointLimits.h"
#include "../controllers/OrdinaryPoseController.h"
//Robot Kinematics
#include "../robot_dh/Schunk.h" 
//For M_PI_2
#define _USE_MATH_DEFINES
#include <cmath>
//For std::setprecision()
#include <iomanip> // For std::setprecision()

using namespace Eigen;
using namespace DQ_robotics;

void control(Matrix<double,8,8> kp, Matrix<double,7,1> thetas, DQ_kinematics robot, DQ eff_pose_reference, DQ_controller& controller);

int main(void)
{
    //Uncomment the next line to show more digits when printing variables.
    //std::cout << std::setprecision(51);

    const double pi2 = M_PI_2;

    //Gain Matrix
	Matrix<double,8,8> kp = Matrix<double,8,8>::Zero(8,8);
	Matrix<double,8,1> kp_diagonal(8,1);
    kp_diagonal << 0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8;
    kp.diagonal() = kp_diagonal;

    //Initial Joint Values
    Matrix<double,7,1> thetas;
    thetas << 0,pi2,0,0,0,0,0;

    std::cout << std::endl << "Initial Thetas" << std::endl << thetas << std::endl;

    //End Effector Pose eff_pose_reference
    DQ eff_pose_reference(1,0,0,0,0,0,0,0.652495);

 	  //Robot DH
	  DQ_kinematics schunk = SchunkKinematics();
    std::cout << std::endl << "Robot initial FKM " << schunk.fkm(thetas) << std::endl;

    //HInfinity Controller
    //std::cout << std::endl <<"H-Inf Robust Controller" << std::endl;
    //HInfinityRobustController r_controller(schunk, kp);
    //control(kp, thetas, schunk, eff_pose_reference, r_controller);
    
    //Ordinary Pose Controller
    std::cout << std::endl <<"Ordinary Pose Controller" << std::endl;
    OrdinaryPoseController op_controller(schunk, kp, 0.01);
    control(kp, thetas, schunk, eff_pose_reference, op_controller);

    //Chiaverini Controller
    std::cout << std::endl <<"Chiaverini's Singularity-Robust Controller" << std::endl;
    DampedNumericalFilteredController c_controller(schunk, kp, 0.01, 0.01, 0.01);
    control(kp, thetas, schunk,eff_pose_reference, c_controller);

    //Chiaverini Controller With Joint Limit Considerantion
    //Upper Limits
    Matrix<double,7,1> upper_limits;
    upper_limits << pi2,pi2,pi2,pi2,pi2,pi2,pi2;
    //Lower Limits
    Matrix<double,7,1> lower_limits;
    lower_limits << -pi2,-pi2,-pi2,-pi2,-pi2,-pi2,-pi2;  

    std::cout << std::endl <<"Chiaverini's Singularity-Robust Controller with Joint Limit Consideration" << std::endl;
    DampedNumericalFilteredControllerJointLimits cjl_controller(schunk, upper_limits, lower_limits, kp, 0.01, 0.01, 0.001);
    control(kp,thetas, schunk, eff_pose_reference, cjl_controller );

    return 0;
}

void control(Matrix<double,8,8> kp, Matrix<double,7,1> thetas, DQ_kinematics robot, DQ eff_pose_reference, DQ_controller& controller)
{

    //Control Loop Variables
    DQ eff_pose_current(0);
    DQ eff_pose_difference(20); //An initial large value so it does not break from the control loop
    double control_threshold = 1.e-4;
    int control_step_count=0;

    Matrix<double,7,1> delta_thetas; //Only necessary for performance measurements commented bellow

    //Control Loop
    while(eff_pose_difference.vec8().norm() > control_threshold)
    {   

        //One controller step
        delta_thetas = controller.getNewJointVelocities(eff_pose_reference,thetas);
        thetas = thetas + delta_thetas;

        //End of control check
        eff_pose_current = robot.fkm(thetas);
        eff_pose_difference = (eff_pose_current - eff_pose_reference);

        //The following are optional convergence and efficiency measurements.        
        //std::cout << std::endl << "error: " << eff_pose_difference.vec8().norm() << std::endl; //This one should fall exponentially.
        //std::cout << std::endl << "biggest d_theta: " << delta_thetas.maxCoeff() << std::endl; //This should be as small as possible.
        //std::cout << std::endl << "biggest theta:   " << thetas.maxCoeff() << std::endl;       //This should be inside the inverval between manipulator joint limits.

        //Count Steps
        control_step_count++;

    }

    std::cout << std::endl <<"Control Loop Ended In " << control_step_count << " Steps" << std::endl;
    std::cout << std::endl <<"End Effector Pose Reference" << std::endl << eff_pose_reference << std::endl;
    std::cout << std::endl <<"End Effector Final Pose" << std::endl << eff_pose_current << std::endl;
    std::cout << std::endl <<"End Effector Final Pose Difference" << std::endl << eff_pose_difference << std::endl;
    std::cout << std::endl <<"Final Thetas" << std::endl << thetas << std::endl;
}


