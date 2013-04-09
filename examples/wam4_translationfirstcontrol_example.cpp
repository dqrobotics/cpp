/**
Schunk Control Examples Using TranslationFirstPoseController.h



\author Murilo Marques Marinho
\since 04/2013

*/

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../DQ_controller.h"
#include "../controllers/TranslationFirstPoseController.h" //Has TranslationFirstPoseController(...,...)
#include "../robot_dh/WAM4.h" //Has Wam4Kinematics()

#define _USE_MATH_DEFINES
#include <cmath> //For fabs and M_PI_2

using namespace Eigen;
using namespace DQ_robotics;

int main(void)
{
    const double pi2 = M_PI_2;

    //Gain Matrix
	Matrix<double,4,4> kp = Matrix<double,4,4>::Zero(4,4);
	Matrix<double,4,1> kp_diagonal(4,1);
    kp_diagonal << .5,.5,.5,.5;
    kp.diagonal() = kp_diagonal;

	Matrix<double,4,4> kr = Matrix<double,4,4>::Zero(4,4);
	Matrix<double,4,1> kr_diagonal(4,1);
	kr_diagonal << 0.2,0.2,0.2,0.2;
	kr.diagonal() = kr_diagonal;

    //Initial Joint Values
	Matrix<double,4,1> thetas; // Wam4
	thetas << 0,-pi2/4,pi2,0;  // Wam4

    //End Effector Pose eff_pose_reference
    DQ eff_pose_reference(1,0,0,0,0,0,0,0.2750000000000000222044604);

 	//Robot DH
	DQ_kinematics robot = Wam4Kinematics();

    //TranslationFirstPose Controller
    std::cout << std::endl <<"TranslationFirstPose Controller" << std::endl;
    TranslationFirstPoseController pf_controller(robot, kp, kr, 0.0001, 0.0001);

    //Control Loop Variables
    DQ eff_pose_current(0);
	DQ eff_pose_older = robot.fkm(thetas);
    DQ eff_pose_difference(0); 

	DQ eff_translation_difference(20); //Big initial values
	DQ eff_rotation_difference(20);    //so they don't break the loop.

    double translation_threshold = 1.e-5;
	double rotation_threshold    = 1.e-3;

    int control_step_count = 0;

    //Control Loop
	//The control loop end criterion is changed in this example. This criterion is used 
	//to search for local convergence. Both sensibilities can be changed using the given thresholds.
    while( (eff_translation_difference.vec8().norm() > translation_threshold) ||
		   (eff_rotation_difference.vec8().norm()    > rotation_threshold   )    )
    {   

        //One controller step
        thetas = pf_controller.getNewJointPositions(eff_pose_reference,thetas);

        //End of control check
        eff_pose_current = robot.fkm(thetas);
        eff_pose_difference = (eff_pose_current - eff_pose_older);
		eff_pose_older = eff_pose_current;

		// Get translation difference
		eff_translation_difference = eff_pose_current.translation() - eff_pose_older.translation();
		// Get rotation difference
		eff_rotation_difference    = P(eff_pose_difference);

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



