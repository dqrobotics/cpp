/**
(C) Copyright 2011-2025 DQ Robotics Developers

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
1. Murilo M. Marinho (murilomarinho@ieee.org)
    - Responsible for the original implementation.

2. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
    - Added the get_supported_joint_types() method.
*/

#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDenso.h>
namespace DQ_robotics
{

DQ_SerialManipulatorDenso::DQ_SerialManipulatorDenso(const MatrixXd& denso_matrix):
    DQ_SerialManipulator(denso_matrix.cols())
{
    if(denso_matrix.rows() != 6)
    {
        throw(std::range_error("Bad DQ_SerialManipulatorDenso(MatrixXd) call: denso_matrix should be 6xn"));
    }
    denso_matrix_ = denso_matrix;
}

DQ DQ_SerialManipulatorDenso::_denso2dh(const double &q, const int &ith) const
{
    const double& a = denso_matrix_(0,ith);
    const double& b = denso_matrix_(1,ith);
    const double& d = denso_matrix_(2,ith);
    const double& alpha = denso_matrix_(3,ith);
    const double& beta = denso_matrix_(4,ith);
    const double& gamma = denso_matrix_(5,ith);

    DQ z_rot = cos((gamma + q)/2.0) + k_*sin((gamma + q)/2.0);
    DQ q_t     = 1.0+0.5*E_*(a*i_+b*j_+d*k_);
    DQ q_alpha = cos(alpha/2.)+i_*sin(alpha/2.);
    DQ q_beta  = cos(beta/2.)+j_*sin(beta/2.);

    return z_rot*q_t*q_alpha*q_beta;
}

/**
 * @brief DQ_SerialManipulatorDenso::get_supported_joint_types gets the supported joint types.
 * @return A vector containing the supported joint types.
 */
std::vector<DQ_JointType> DQ_SerialManipulatorDenso::get_supported_joint_types() const
{
    return {DQ_JointType::REVOLUTE};
}

VectorXd DQ_SerialManipulatorDenso::get_as() const
{
    return denso_matrix_.row(0);
}

VectorXd DQ_SerialManipulatorDenso::get_bs() const
{
    return denso_matrix_.row(1);
}

VectorXd DQ_SerialManipulatorDenso::get_ds() const
{
    return denso_matrix_.row(2);
}

VectorXd DQ_SerialManipulatorDenso::get_alphas() const
{
    return denso_matrix_.row(3);
}

VectorXd DQ_SerialManipulatorDenso::get_betas() const
{
    return denso_matrix_.row(4);
}

VectorXd DQ_SerialManipulatorDenso::get_gammas() const
{
    return denso_matrix_.row(5);
}

DQ  DQ_SerialManipulatorDenso::raw_fkm(const VectorXd& q_vec, const int& to_ith_link) const
{
    _check_q_vec(q_vec);
    _check_to_ith_link(to_ith_link);

    DQ q(1);
    int j = 0;
    for (int i = 0; i < (to_ith_link+1); i++) {
        q = q * _denso2dh(q_vec(i-j), i);
    }
    return q;
}

MatrixXd DQ_SerialManipulatorDenso::raw_pose_jacobian(const VectorXd &q_vec, const int &to_ith_link) const
{
    _check_q_vec(q_vec);
    _check_to_ith_link(to_ith_link);

    MatrixXd J = MatrixXd::Zero(8,to_ith_link+1);
    DQ x_effector = raw_fkm(q_vec,to_ith_link);

    DQ x(1);

    for(int i=0;i<to_ith_link+1;i++)
    {
        const DQ w = k_;
        const DQ z = 0.5*Ad(x,w);
        x = x*_denso2dh(q_vec(i),i);
        const DQ j = z * x_effector;
        J.col(i)= vec8(j);
    }
    return J;
}

/**
 * @brief This method returns the first to_ith_link columns of the time derivative of the pose Jacobian.
 *        The base displacement and the effector are not taken into account.
 * @param q. VectorXd representing the robot joint configuration.
 * @param q_dot. VectorXd representing the robot joint velocities.
 * @param to_ith_link. The index to a link. This defines until which link the pose_jacobian_derivative
 *                     will be calculated.
 * @returns a MatrixXd representing the first to_ith_link columns of the desired Jacobian derivative.
 *
 */
MatrixXd DQ_SerialManipulatorDenso::raw_pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot, const int &to_ith_link) const
{
    _check_q_vec(q);
    _check_q_vec(q_dot);
    _check_to_ith_link(to_ith_link);

    int n = to_ith_link+1;
    DQ x_effector = raw_fkm(q,to_ith_link);
    MatrixXd J    = raw_pose_jacobian(q,to_ith_link);
    VectorXd vec_x_effector_dot = J*q_dot.head(n);
    DQ x = DQ(1);
    MatrixXd J_dot = MatrixXd::Zero(8,n);

    for(int i=0;i<n;i++)
    {
        const DQ w = k_;
        const DQ z = 0.5*x*w*conj(x);

        VectorXd vec_zdot;
        if(i==0)
        {
            vec_zdot = VectorXd::Zero(8,1);
        }
        else
        {
            vec_zdot = 0.5*(haminus8(w*conj(x)) + hamiplus8(x*w)*C8())*raw_pose_jacobian(q,i-1)*q_dot.head(i);
        }
        J_dot.col(i) = haminus8(x_effector)*vec_zdot + hamiplus8(z)*vec_x_effector_dot;
        x = x*_denso2dh(q(i),i);
    }

    return J_dot;
}



}
