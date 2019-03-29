/**
(C) Copyright 2011-2018 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Murilo M. Marinho        (murilo@nml.t.u-tokyo.ac.jp)
- Mateus Rodrigues Martins (martinsrmateus@gmail.com)
*/

#ifndef DQ_KINEMATICS_H
#define DQ_KINEMATICS_H

#include <dqrobotics/DQ.h>

#include <math.h>       //library for math functions
#include <stdexcept>    //For range_error
#include <eigen3/Eigen/Dense>  //Library for matrix usage
#include <limits>       //Used in pseudoinverse()
#include <string>

using namespace Eigen;

namespace DQ_robotics
{



class DQ_kinematics{

    // private attributtes
private:

    //Para uso nas funções Jacobian...
    MatrixXd    dh_matrix_;
    std::string dh_matrix_convention_;

    DQ curr_base_;
    DQ curr_effector_;

    // public methods
public:
    // Class constructors: Creates a Dual Quaternion as a DQ object.

    DQ_kinematics(const MatrixXd& dh_matrix, const std::string& convention = "standard" );

    DQ_kinematics(){};

    ~DQ_kinematics();


    /*
        * Public constant methods: Can be called by DQ_kinematics objects.
        * To use these methods, type: 'dq_kinematics_object.method_name();' where 'method_name' is the name of one of the methods below.
        * Or in another way type: 'DQ_kinematics::method_name(dq_kinematics_object);' that works well too.
        * For displaying the results of methods, the DISPLAY and MATRIX functions of DQ class can be used
        */

    MatrixXd getDHMatrix();

    int n_link() const;

    VectorXd theta() const;

    VectorXd d() const;

    VectorXd a() const;

    VectorXd alpha() const;

    VectorXd dummy() const;
    void set_dummy( const VectorXd& dummy_vector);

    int n_dummy() const;

    std::string convention() const;

    DQ base() const;
    DQ set_base( const DQ& new_base);

    DQ effector() const;
    DQ set_effector( const DQ& new_effector);

    DQ raw_fkm( const VectorXd& theta_vec) const;
    DQ raw_fkm( const VectorXd& theta_vec, const int& ith) const;

    DQ fkm( const VectorXd& theta_vec) const;
    DQ fkm( const VectorXd& theta_vec, const int& ith) const;

    DQ dh2dq( const double& theta_ang, const int& link_i) const;

    DQ get_z( const VectorXd& q) const;

    MatrixXd pose_jacobian(           const VectorXd& theta_vec, const int& to_link) const;
    MatrixXd pose_jacobian(           const VectorXd& theta_vec) const;
    MatrixXd raw_pose_jacobian(       const VectorXd& theta_vec, const int& to_link) const;
    MatrixXd pose_jacobian_derivative( const VectorXd& theta_vec, const VectorXd& theta_vec_dot, const int& to_link) const;

    ///DEPRECATED SIGNATURES
    DEPRECATED MatrixXd analyticalJacobian( const VectorXd& theta_vec) const;
    DEPRECATED MatrixXd jacobian(           const VectorXd& theta_vec, const int& to_link) const;
    DEPRECATED MatrixXd jacobian(           const VectorXd& theta_vec) const;
    DEPRECATED MatrixXd raw_jacobian(       const VectorXd& theta_vec, const int& to_link) const;
    DEPRECATED MatrixXd jacobianDerivative( const VectorXd& theta_vec, const VectorXd& theta_vec_dot, const int& to_link) const;
    DEPRECATED int links() const;

};


int n_link( const DQ_kinematics& dq_kin);

VectorXd theta( const DQ_kinematics& dq_kin);

VectorXd d( const DQ_kinematics& dq_kin);

VectorXd a( const DQ_kinematics& dq_kin);

VectorXd alpha( const DQ_kinematics& dq_kin);

VectorXd dummy( const DQ_kinematics& dq_kin);

int n_dummy( const DQ_kinematics& dq_kin);

std::string convention( const DQ_kinematics& dq_kin);

DQ base( const DQ_kinematics& dq_kin);

DQ effector( const DQ_kinematics& dq_kin);

DQ set_base( DQ_kinematics& dq_kin, const DQ& new_base);

DQ set_effector( DQ_kinematics& dq_kin, const DQ& new_effector);

DQ raw_fkm( const DQ_kinematics& dq_kin, const VectorXd& theta_vec);
DQ raw_fkm( const DQ_kinematics& dq_kin, const VectorXd& theta_vec, const int& ith);

DQ dh2dq( const DQ_kinematics& dq_kin, const double& theta_ang, const int& link_i);

DQ get_z( const DQ_kinematics& dq_kin, const VectorXd& q);

MatrixXd pose_jacobian( const DQ_kinematics& dq_kin,   const VectorXd& theta_vec );
MatrixXd pose_jacobian( const DQ_kinematics& dq_kin,   const VectorXd& theta_vec,  const int &to_link);
MatrixXd raw_pose_jacobian( const DQ_kinematics& dq_kin,   const VectorXd& theta_vec, const int& to_link);
MatrixXd pose_jacobian_derivative( const DQ_kinematics& dq_kin,   const VectorXd& theta_vec, const VectorXd& theta_vec_dot, const int& to_link);

MatrixXd rotation_jacobian(const MatrixXd& pose_jacobian);

MatrixXd translation_jacobian( const MatrixXd& pose_jacobian, const DQ& x);

MatrixXd distance_jacobian( const DQ_kinematics& dq_kin, const MatrixXd& param_jacobian, const DQ& x);

MatrixXd pseudo_inverse( const MatrixXd& matrix);

///DEPRECATED SIGNATURES
DEPRECATED MatrixXd analyticalJacobian( const DQ_kinematics& dq_kin,   const VectorXd& theta_vec);
DEPRECATED MatrixXd jacobian(           const DQ_kinematics& dq_kin,   const VectorXd& theta_vec,  const int &to_link);
DEPRECATED MatrixXd jacobian(           const DQ_kinematics& dq_kin,   const VectorXd& theta_vec); //The MATLAB syntax, kept for legacy reasons.
DEPRECATED MatrixXd jacobianDerivative( const DQ_kinematics& dq_kin,   const VectorXd& theta_vec, const VectorXd& theta_vec_dot, const int& to_link);
DEPRECATED MatrixXd raw_jacobian(       const DQ_kinematics& dq_kin,   const VectorXd& theta_vec, const int& to_link);
DEPRECATED MatrixXd rotationJacobian(   const MatrixXd& pose_jacobian);
DEPRECATED MatrixXd translationJacobian(const MatrixXd& pose_jacobian, const DQ& x);
DEPRECATED MatrixXd jacobp(             const MatrixXd& pose_jacobian, const DQ& x); //The MATLAB syntax, kept for legacy reasons.
DEPRECATED MatrixXd distanceJacobian(   const MatrixXd& param_jacobian, const DQ& x);
DEPRECATED MatrixXd jacobd(             const MatrixXd& param_jacobian, const DQ& x);
DEPRECATED MatrixXd pseudoInverse(      const MatrixXd& matrix);
DEPRECATED int      links();

}//Namespace DQRobotics

#endif // DQ_KINEMATICS_H_INCLUDED
