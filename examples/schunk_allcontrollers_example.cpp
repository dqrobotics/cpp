/**
Schunk Control Examples Using HInfinityRobustController.h



\author Murilo Marques Marinho
\since 01/2013

*/

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../DQ_controller.h"
#include "../controllers/HInfinityRobustController.h" //Has HInfinityRobustController(...,...)
#include "../controllers/DampedNumericalFilteredController.h"
#include "../robot_dh/Schunk.h" //Has SchunkKinematics()

#define _USE_MATH_DEFINES
#include <cmath> //For fabs and M_PI_2

using namespace Eigen;
using namespace DQ_robotics;



void control(Matrix<double,8,8> kp, Matrix<double,7,1> thetas, DQ_kinematics robot, DQ eff_pose_reference, DQ_controller& controller);

int main(void)
{
    const double pi2 = M_PI_2;

    //Gain Matrix
	Matrix<double,8,8> kp = Matrix<double,8,8>::Zero(8,8);
	Matrix<double,8,1> kp_diagonal(8,1);
    kp_diagonal << 0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8;
    kp.diagonal() = kp_diagonal;

    //Initial Joint Values
    Matrix<double,7,1> thetas;
    thetas << 0,pi2,0,0,0,0,0;

    //End Effector Pose eff_pose_reference
    DQ eff_pose_reference(1,0,0,0,0,0,0,0.652495);

 	//Robot DH
	DQ_kinematics schunk = SchunkKinematics();

    //HInfinity Controller
    std::cout << std::endl <<"H-Inf Robust Controller" << std::endl;
    HInfinityRobustController r_controller(schunk, kp);
    control(kp, thetas, schunk, eff_pose_reference, r_controller);
    
    //Chiaverini Controller
    std::cout << std::endl <<"Chiaverini Robust Controller" << std::endl;
    DampedNumericalFilteredController c_controller(schunk, kp, 0.01, 0.01, 0.001);
    control(kp, thetas, schunk,eff_pose_reference, c_controller);

    return 0;
}

void control(Matrix<double,8,8> kp, Matrix<double,7,1> thetas, DQ_kinematics robot, DQ eff_pose_reference, DQ_controller& controller)
{

    //Control Loop Variables
    DQ eff_pose_current(0);
    DQ eff_pose_difference(0);
    double control_threshold = 1.e-5;
    bool continue_control = true;
    int control_step_count=0;

    //Control Loop
    while(continue_control)
    {   
        //std::cout << std::endl <<"thetas: " << thetas << std::endl;

        //One controller step
        thetas = controller.getNewJointPositions(eff_pose_reference,thetas);

        //End of control check
        eff_pose_current = robot.fkm(thetas);
        eff_pose_difference = (eff_pose_current - eff_pose_reference);
        continue_control = false;
        for(int i=0;i<8;++i)
        {

            if(fabs(eff_pose_difference.q(i,0)) > control_threshold)
                continue_control = true;

        }

        //Count Steps
        control_step_count++;
    }

    std::cout << std::endl <<"Control Loop Ended In " << control_step_count << " Steps" << std::endl;
    std::cout << std::endl <<"End Effector Pose Reference" << std::endl << eff_pose_reference << std::endl;
    std::cout << std::endl <<"End Effector Final Pose" << std::endl << eff_pose_current << std::endl;
    std::cout << std::endl <<"End Effector Final Pose Difference" << std::endl << eff_pose_difference << std::endl;
    std::cout << std::endl <<"Final Thetas" << std::endl << thetas << std::endl;
}


