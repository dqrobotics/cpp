#ifndef DQORDINARYPOSECONTROLLER_H
#define DQORDINARYPOSECONTROLLER_H

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../DQ_controller.h"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;


namespace DQ_robotics
{



class OrdinaryPoseController : public DQ_controller
{

public: //variables

private: //variables

    DQ_kinematics robot_;
    int robot_dofs_;

    MatrixXd kp_;
    double lambda_;

    VectorXd thetas_;
    VectorXd delta_thetas_;

    VectorXd error_;

    MatrixXd task_jacobian_;
    MatrixXd task_jacobian_pseudoinverse_;

    MatrixXd identity_;

    DQ end_effector_pose_;

    DQ dq_one_;


public: //methods
    OrdinaryPoseController(DQ_kinematics robot, MatrixXd feedback_gain, double lambda);
    ~OrdinaryPoseController(){};

    VectorXd getNewJointPositions( const DQ reference, const VectorXd thetas);
    VectorXd getNewJointVelocities( const DQ reference, const VectorXd thetas);

private: //methods

};



}


#endif
