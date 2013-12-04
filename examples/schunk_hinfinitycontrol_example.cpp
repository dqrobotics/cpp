/**
Schunk Control Examples Using HInfinityRobustController.h



\author Murilo Marques Marinho
\since 01/2013

*/

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../controllers/HInfinityRobustController.h" //Has HInfinityRobustController(...,...)
#include "../robot_dh/Schunk.h" //Has SchunkKinematics()

#include <cmath> //For fabs and M_PI_2

using namespace Eigen;
using namespace DQ_robotics;

int main(void)
{
    const double pi2 = M_PI_2;

    //Gain Matrix
	  Matrix<double,8,1> B;
    B << 1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0;

    //Initial Joint Values
    Matrix<double,7,1> initial_thetas;
    initial_thetas << 0.1,0.1,0.1,0.1,0.1,0.1,0.1;
    Matrix<double,7,1> thetas = initial_thetas;

 	  //Robot DH
	  DQ_kinematics schunk = SchunkKinematics();

    //End Effector Pose eff_pose_reference
    Matrix<double,7,1> reference_thetas;
    reference_thetas << 0.2,0.2,0.2,0.2,0.2,0.2,0.2;
    DQ eff_pose_reference( schunk.fkm( reference_thetas ) );

    //HInfinity Controller
    double alpha = 2.0;

    //Gamma finder
    double initial_gamma  =  0.01;
    double gamma_step     =  0.001;
    double final_gamma    =  3.00;
    int    maximum_steps  =  10;
    double current_gamma  =  initial_gamma;
    bool   finished       =  false;


    //Control Loop Variables
    DQ eff_pose_current(0);
    DQ eff_pose_difference(20);
    double control_threshold = 1.e-10;
    int control_step_count   = 0;

    while( current_gamma < final_gamma && not finished )
    {
      HInfinityRobustController controller = HInfinityRobustController(schunk, B, current_gamma, alpha);

      //Control Loop Variables
      eff_pose_current    = DQ(0);
      eff_pose_difference = DQ(20);
      control_step_count  = 0;
      thetas = initial_thetas;

      finished = true;
      //Control Loop
      while(eff_pose_difference.vec8().norm() > control_threshold)
      {   
          //One controller step
          thetas = controller.getNewJointPositions(eff_pose_reference,thetas);

          //End of control check
          eff_pose_current = schunk.fkm(thetas);
          eff_pose_difference = (eff_pose_current - eff_pose_reference);
          //std::cout << std::endl << (eff_pose_difference.vec8()).transpose().norm() << std::endl;

          //Count Steps
          control_step_count++;
          if(control_step_count > maximum_steps)
          {
            std::cout << std::endl << "Gamma " << current_gamma << " not fit." << std::endl;
            break;
          }
      }      
      if(control_step_count > maximum_steps)
        finished = false;

      current_gamma += gamma_step;
    }
    std::cout << std::endl << "Gamma " << current_gamma << " selected." << std::endl; 


    std::cout << std::endl <<"Control Loop Ended In " << control_step_count << " Steps" << std::endl;
    std::cout << std::endl <<"End Effector Pose Reference" << std::endl << eff_pose_reference << std::endl;
    std::cout << std::endl <<"End Effector Final Pose" << std::endl << eff_pose_current << std::endl;
    std::cout << std::endl <<"End Effector Final Pose Difference" << std::endl << eff_pose_difference << std::endl;
    std::cout << std::endl <<"Final Thetas" << std::endl << thetas << std::endl;

    return 0;
}


