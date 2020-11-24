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

#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>

namespace DQ_robotics
{

DQ_SerialManipulatorDH::DQ_SerialManipulatorDH(const MatrixXd& dh_matrix):
    DQ_SerialManipulator(dh_matrix.cols())
{
    if(dh_matrix.rows() != 5)
    {
        throw(std::range_error("Bad DQ_SerialManipulatorDH(dh_matrix, convention) call: dh_matrix should be 5xn"));
    }
    dh_matrix_ = dh_matrix;
}

DQ DQ_SerialManipulatorDH::_dh2dq(const double &q, const int &ith) const
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
                sine_of_half_alpha*sine_of_half_theta,
                cosine_of_half_alpha*sine_of_half_theta,
                -(a*sine_of_half_alpha*cosine_of_half_theta) /2.0 - (d*cosine_of_half_alpha*sine_of_half_theta)/2.0,
                (a*cosine_of_half_alpha*cosine_of_half_theta)/2.0 - (d*sine_of_half_alpha*sine_of_half_theta  )/2.0,
                (a*cosine_of_half_alpha*sine_of_half_theta)  /2.0 + (d*sine_of_half_alpha*cosine_of_half_theta)/2.0,
                (d*cosine_of_half_alpha*cosine_of_half_theta)/2.0 - (a*sine_of_half_alpha*sine_of_half_theta  )/2.0
                );
}

DQ DQ_SerialManipulatorDH::_get_w(const int &ith) const
{
    const int joint_type = int(dh_matrix_(4,ith));
    if(joint_type == JOINT_ROTATIONAL)
        return k_;
    else
        return E_*k_;
}

VectorXd  DQ_SerialManipulatorDH::get_thetas() const
{
    return dh_matrix_.row(0);
}


VectorXd  DQ_SerialManipulatorDH::get_ds() const
{
    return dh_matrix_.row(1);
}

VectorXd  DQ_SerialManipulatorDH::get_as() const
{
    return dh_matrix_.row(2);
}

VectorXd  DQ_SerialManipulatorDH::get_alphas() const
{
    return dh_matrix_.row(3);
}

VectorXd DQ_SerialManipulatorDH::get_types() const
{
    return dh_matrix_.row(4);
}


DQ  DQ_SerialManipulatorDH::raw_fkm(const VectorXd& q_vec, const int& to_ith_link) const
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

MatrixXd DQ_SerialManipulatorDH::raw_pose_jacobian(const VectorXd &q_vec, const int &to_ith_link) const
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
