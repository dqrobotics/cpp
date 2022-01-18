/**
(C) Copyright 2020 DQ Robotics Developers

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
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
*/

#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>

namespace DQ_robotics
{

DQ_SerialManipulatorMDH::DQ_SerialManipulatorMDH(const MatrixXd& dh_matrix):
    DQ_SerialManipulator(dh_matrix.cols())
{
    if(dh_matrix.rows() != 5)
    {
        throw(std::range_error("Bad DQ_SerialManipulatorDH(dh_matrix) call: dh_matrix should be 5xn"));
    }
    dh_matrix_ = dh_matrix;
}

DQ_SerialManipulatorMDH::DQ_SerialManipulatorMDH(const MatrixXd &dh_matrix, const std::string&):
    DQ_SerialManipulator(dh_matrix.cols())
{
    if(dh_matrix.rows() != 5)
    {
        throw(std::range_error("Bad DQ_SerialManipulatorMDH(dh_matrix, convention) call: dh_matrix should be 5xn"));
    }
    dh_matrix_ = dh_matrix;
}

DQ DQ_SerialManipulatorMDH::_dh2dq(const double &q, const int &ith) const
{
    double half_theta = dh_matrix_(0,ith)/2.0;
    double d = dh_matrix_(1,ith);
    const double &a = dh_matrix_(2,ith);
    const double half_alpha = dh_matrix_(3,ith)/2.0;
    const int joint_type = int(dh_matrix_(4,ith));

    // Add the effect of the joint value
    if(joint_type == JOINT_ROTATIONAL)
    {
        half_theta = half_theta + (q/2.0);
    }
    else
    {
        d = d + q;
    }

    // Pre-calculate cosines and sines
    const double sine_of_half_theta = sin(half_theta);
    const double cosine_of_half_theta = cos(half_theta);
    const double sine_of_half_alpha = sin(half_alpha);
    const double cosine_of_half_alpha = cos(half_alpha);

    // Return the optimized standard dh2dq calculation
    return DQ(
                cosine_of_half_alpha*cosine_of_half_theta,
                sine_of_half_alpha*cosine_of_half_theta,
                -sine_of_half_alpha*sine_of_half_theta, //MDH
                cosine_of_half_alpha*sine_of_half_theta,
                -(a*sine_of_half_alpha*cosine_of_half_theta) /2.0 - (d*cosine_of_half_alpha*sine_of_half_theta)/2.0,
                (a*cosine_of_half_alpha*cosine_of_half_theta)/2.0 - (d*sine_of_half_alpha*sine_of_half_theta  )/2.0,
                -(a*cosine_of_half_alpha*sine_of_half_theta)  /2.0 - (d*sine_of_half_alpha*cosine_of_half_theta)/2.0, //MDH
                (d*cosine_of_half_alpha*cosine_of_half_theta)/2.0 - (a*sine_of_half_alpha*sine_of_half_theta  )/2.0
                );
}

DQ DQ_SerialManipulatorMDH::_get_w(const int &ith) const
{
    const int joint_type = int(dh_matrix_(4,ith));
    const double alpha = dh_matrix_(3,ith);
    const double &a = dh_matrix_(2,ith);
    if(joint_type == JOINT_ROTATIONAL)
        return -j_*sin(alpha)+ k_*cos(alpha)- E_*a*(j_*cos(alpha) + k_*sin(alpha));
    else
        return E_*(cos(alpha)*k_ - sin(alpha)*j_);
}

VectorXd  DQ_SerialManipulatorMDH::get_thetas() const
{
    return dh_matrix_.row(0);
}


VectorXd  DQ_SerialManipulatorMDH::get_ds() const
{
    return dh_matrix_.row(1);
}

VectorXd  DQ_SerialManipulatorMDH::get_as() const
{
    return dh_matrix_.row(2);
}

VectorXd  DQ_SerialManipulatorMDH::get_alphas() const
{
    return dh_matrix_.row(3);
}

VectorXd DQ_SerialManipulatorMDH::get_types() const
{
    return dh_matrix_.row(4);
}

MatrixXd DQ_SerialManipulatorMDH::pose_jacobian_derivative(const VectorXd &q_vec, const VectorXd &q_vec_dot, const int &to_ith_link) const
{
    _check_q_vec(q_vec);
    _check_q_vec(q_vec_dot);
    _check_to_ith_link(to_ith_link);

    int n = to_ith_link+1;
    DQ x_effector = raw_fkm(q_vec,to_ith_link);
    MatrixXd J    = raw_pose_jacobian(q_vec,to_ith_link);
    VectorXd vec_x_effector_dot = J*q_vec_dot.head(to_ith_link);

    DQ x = DQ(1);
    MatrixXd J_dot = MatrixXd::Zero(8,n);
    int jth=0;

    for(int i=0;i<n;i++)
    {
        const DQ w = _get_w(i);
        const DQ z = 0.5*x*w*conj(x);

        VectorXd vec_zdot;
        if(i==0)
        {
            vec_zdot = VectorXd::Zero(8,1);
        }
        else
        {
            vec_zdot = 0.5*(haminus8(w*conj(x)) + hamiplus8(x*w)*C8())*raw_pose_jacobian(q_vec,i-1)*q_vec_dot.head(i);
        }

        J_dot.col(jth) = haminus8(x_effector)*vec_zdot + hamiplus8(z)*vec_x_effector_dot;
        x = x*_dh2dq(q_vec(jth),i);
        jth = jth+1;
    }

    return J_dot;
}

MatrixXd DQ_SerialManipulatorMDH::pose_jacobian_derivative(const VectorXd &q_vec, const VectorXd &q_vec_dot) const
{
    return pose_jacobian_derivative(q_vec, q_vec_dot, get_dim_configuration_space()-1);
}


DQ  DQ_SerialManipulatorMDH::raw_fkm(const VectorXd& q_vec, const int& to_ith_link) const
{
    _check_q_vec(q_vec);
    _check_to_ith_link(to_ith_link);

    DQ q(1);
    int j = 0;
    for (int i = 0; i < (to_ith_link+1); i++) {
        q = q * _dh2dq(q_vec(i-j), i);
    }
    return q;
}

MatrixXd DQ_SerialManipulatorMDH::raw_pose_jacobian(const VectorXd &q_vec, const int &to_ith_link) const
{
    _check_q_vec(q_vec);
    _check_to_ith_link(to_ith_link);

    MatrixXd J = MatrixXd::Zero(8,to_ith_link+1);
    DQ x_effector = raw_fkm(q_vec,to_ith_link);

    DQ x(1);

    for(int i=0;i<to_ith_link+1;i++)
    {
        DQ w = _get_w(i);
        DQ z = 0.5*Ad(x,w);
        x = x*_dh2dq(q_vec(i),i);
        DQ j = z * x_effector;
        J.col(i)= vec8(j);
    }
    return J;
}


}
