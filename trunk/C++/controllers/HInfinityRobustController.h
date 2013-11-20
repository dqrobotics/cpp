#ifndef DQHINFINITYROBUSTCONTROLLER_H
#define DQHINFINITYROBUSTCONTROLLER_H

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../DQ_controller.h"
#include <Eigen/Dense>

using namespace Eigen;


namespace DQ_robotics
{



class HInfinityRobustController : public DQ_controller
{

public: //variables

private: //variables

    DQ_kinematics robot_;
    int robot_dofs_;

    MatrixXd kp_;

    VectorXd thetas_;
    VectorXd delta_thetas_;

    VectorXd reference_state_variables_;
    VectorXd measured_state_variables_;

    VectorXd error_;

    Matrix<double,8,8> C8_;

    MatrixXd N_;

    MatrixXd task_jacobian_;
    MatrixXd N_pseudoinverse_;

    DQ end_effector_pose_;


public: //methods
    HInfinityRobustController(DQ_kinematics robot, MatrixXd feedback_gain);
    ~HInfinityRobustController(){};

    VectorXd getNewJointPositions( const DQ reference, const VectorXd thetas);
    VectorXd getNewJointVelocities( const DQ reference, const VectorXd thetas);

private: //methods



};



}


#endif
