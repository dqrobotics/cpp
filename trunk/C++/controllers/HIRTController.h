#ifndef HIRTCONTROLLER_H
#define HIRTCONTROLLER_H

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../DQ_controller.h"
#include <Eigen/Dense>

using namespace Eigen;


namespace DQ_robotics
{



class HIRTController : public DQ_controller
{

public: //variables

private: //variables

    bool runonce_;

    DQ_kinematics robot_;
    int robot_dofs_;

    MatrixXd kp_;
    double gamma_;
    double alpha_;
    Matrix<double,8,1> B_;
    Matrix<double,8,1> Bw_;

    VectorXd thetas_;
    VectorXd delta_thetas_;

    VectorXd reference_state_variables_;
    DQ old_reference_;
    VectorXd measured_state_variables_;

    VectorXd error_;

    Matrix<double,8,8> C8_;
    Matrix<double,8,8> identity8_;

    MatrixXd N_;

    MatrixXd task_jacobian_;
    MatrixXd N_pseudoinverse_;

    DQ end_effector_pose_;


public: //methods
    HIRTController( const DQ_kinematics& robot, const Matrix<double,8,1>& B, const double& gamma, const double& alpha);
    ~HIRTController(){};

    VectorXd getNewJointPositions( const DQ reference, const VectorXd thetas);
    VectorXd getNewJointVelocities( const DQ reference, const VectorXd thetas);

private: //methods



};



}


#endif
