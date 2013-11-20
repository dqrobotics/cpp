/**
* Chiaverini's Singularity Robust controller for the unit dual quaternion space.
*
* \author Murilo Marques Marinho (murilomarinho@lara.unb.br)
* \since 2012/07
***********************************************************
*              REVISION HISTORY
***********************************************************
* YYYY/MM/DD   Author (e-mail address)
*            - Description
***********************************************************
* 2013/11/22   Murilo Marques Marinho (murilomarinho@lara.unb.br)
             - Added derivative and integral gains to the control 
               loop.
             - Added const qualifiers and references whenever
               possible.

* 2013/06/01   Murilo Marques Marinho (murilomarinho@lara.unb.br)
             - Fixed the minimun singular value considered in the 
               numerical filtering. It was considering the last 
               singular value of the matrix when it should be consi-
               dering the 6th.
             - Changed the inversion algorithm in the damped pseudo-
               inverse calculation. It was using .inverse(), which
               is only recomended for matrix.size() <= 4. Changed it
               to use Cholesky decomposition as it is always a posi-
               tive definite matrix.
             - Minor changes in the included files organization.
***********************************************************
*/


#ifndef DQDAMPEDNUMERICALFILTEREDCONTROLLER_H
#define DQDAMPEDNUMERICALFILTEREDCONTROLLER_H

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../DQ_controller.h"
#include <Eigen/Dense>
#include <iostream>

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
    MatrixXd ki_;
    MatrixXd kd_;
    double beta_;
    double lambda_max_;
    double epsilon_;

    VectorXd thetas_;
    VectorXd delta_thetas_;

    VectorXd error_;
    VectorXd integral_error_;
    VectorXd last_error_;
    bool at_least_one_error_;

    MatrixXd task_jacobian_;
    MatrixXd task_jacobian_pseudoinverse_;

    JacobiSVD<MatrixXd> svd_;
    VectorXd singular_values_;
    MatrixXd svd_sigma_inverted_;
    MatrixXd identity_;

    DQ end_effector_pose_;

    DQ dq_one_;


public: //methods
    DampedNumericalFilteredController(){};
    DampedNumericalFilteredController( DQ_kinematics robot, MatrixXd kp, double beta, double lambda_max, double epsilon); //Kept for backwards compatibility
    DampedNumericalFilteredController( const DQ_kinematics& robot, const MatrixXd& kp, const MatrixXd& ki, const MatrixXd& kd, const double& beta, const double& lambda_max, const double& epsilon);
    ~DampedNumericalFilteredController(){};

    VectorXd getNewJointPositions( const DQ reference, const VectorXd thetas);
    VectorXd getNewJointVelocities( const DQ reference, const VectorXd thetas);
    
    void setPGain( const MatrixXd new_kp );
    void setIGain( const MatrixXd new_ki );

private: //methods

};



}


#endif
