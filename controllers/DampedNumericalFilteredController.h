#ifndef DQDAMPEDNUMERICALFILTEREDCONTROLLER_H
#define DQDAMPEDNUMERICALFILTEREDCONTROLLER_H

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../DQ_controller.h"
#include <Eigen/Dense>

using namespace Eigen;


namespace DQ_robotics
{



class DampedNumericalFilteredController : public DQ_controller
{

public: //variables

private: //variables

    DQ_kinematics robot_;
    int robot_dofs_;

    MatrixXd kp_;
    double beta_;
    double lambda_max_;
    double epsilon_;

    VectorXd thetas_;
    VectorXd delta_thetas_;

    VectorXd error_;

    MatrixXd task_jacobian_;
    MatrixXd task_jacobian_pseudoinverse_;

    JacobiSVD<MatrixXd> svd_;
    VectorXd singular_values_;
    MatrixXd svd_sigma_inverted_;
    MatrixXd identity_;

    DQ end_effector_pose_;

    DQ dq_one_;


public: //methods
    DampedNumericalFilteredController(DQ_kinematics robot, MatrixXd feedback_gain, double beta, double lambda_max, double epsilon);
    ~DampedNumericalFilteredController(){};

    VectorXd getNewJointPositions(DQ reference, VectorXd thetas);
    VectorXd getNewJointVelocities(DQ reference, VectorXd thetas);

private: //methods

};



}


#endif
