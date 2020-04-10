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

namespace DQ_robotics
{

DQ_SerialManipulatorDH::DQ_SerialManipulatorDH(const MatrixXd& dh_matrix, const std::string& convention):
    DQ_SerialManipulator(dh_matrix.block(0,0,dh_matrix.rows()-1,dh_matrix.cols()), convention)
{
    if (convention != "standard" && convention != "modified")
    {
        throw(std::range_error("Bad DQ_SerialManipulator(dh_matrix, convention) call: convention must be 'standard' or 'modified' "));
    }
    if (convention == "modified")
    {
        throw(std::runtime_error("Bad DQ_SerialManipulator(dh_matrix, convention) call: the 'modified' convention is not implemented yet"));
    }
    if(dh_matrix.rows() != 5)
    {
        throw(std::range_error("Bad DQ_SerialManipulatorDH(dh_matrix, convention) call: dh_matrix should be 5xn"));
    }
    dh_matrix_ = dh_matrix;
    dh_matrix_convention_ = convention;
}

DQ DQ_SerialManipulatorDH::_dh2dq(const double &q, const int &ith) const
{
    double half_theta = dh_matrix_(0,ith)/2.0;
    double d = dh_matrix_(1,ith);
    const double &a = dh_matrix_(2,ith);
    const double half_alpha = dh_matrix_(3,ith)/2.0;
    const int joint_type = dh_matrix_(4,ith);

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

DQ DQ_SerialManipulatorDH::_dh2dq_dot(const double &q, const int &ith) const
{
    // The optimized standard dh2dq_dot calculation
    // Store half angles and displacements
    double half_theta = dh_matrix_(0,ith)/2.0;
    double d = dh_matrix_(1,ith);
    const double &a = dh_matrix_(2,ith);
    const double half_alpha = dh_matrix_(3,ith)/2.0;
    const int joint_type = dh_matrix_(4,ith);

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

    // Return the optimized dh2dq_dot calculation
    if(joint_type == JOINT_ROTATIONAL)
    {
        return DQ(  -(cosine_of_half_alpha*sine_of_half_theta    )/2.0,
                    -(sine_of_half_alpha*sine_of_half_theta      )/2.0,
                    (sine_of_half_alpha*cosine_of_half_theta    )/2.0,
                    (cosine_of_half_alpha*cosine_of_half_theta  )/2.0,
                    (a*sine_of_half_alpha*sine_of_half_theta    )/4.0 - (d*cosine_of_half_alpha*cosine_of_half_theta)/4.0,
                    -(a*cosine_of_half_alpha*sine_of_half_theta  )/4.0 - (d*sine_of_half_alpha*cosine_of_half_theta  )/4.0,
                    (a*cosine_of_half_alpha*cosine_of_half_theta)/4.0 - (d*sine_of_half_alpha*sine_of_half_theta    )/4.0,
                    -(a*sine_of_half_alpha*cosine_of_half_theta  )/4.0 - (d*cosine_of_half_alpha*sine_of_half_theta  )/4.0
                    );
    }
    else
    {
        return DQ(  0,
                    0,
                    0,
                    0,
                    -(cosine_of_half_alpha*sine_of_half_theta)/2.0,
                    -(sine_of_half_alpha*sine_of_half_theta)/2.0,
                    (sine_of_half_alpha*cosine_of_half_theta)/2.0,
                    (cosine_of_half_alpha*cosine_of_half_theta)/2.0
                    );
    }
}

VectorXd  DQ_SerialManipulatorDH::type() const
{
    VectorXd aux_d(dh_matrix_.cols());
    for (int i = 0; i < dh_matrix_.cols(); i++) {
        aux_d(i) = dh_matrix_(4,i);
    }
    return aux_d;
}

MatrixXd DQ_SerialManipulatorDH::raw_pose_jacobian(const VectorXd &q_vec, const int &to_ith_link) const
{
    MatrixXd J = MatrixXd::Zero(8,to_ith_link+1);

    // The optimized raw_jacobian calculation
    DQ x_forward  = DQ(1);
    DQ x_backward = raw_fkm(q_vec,to_ith_link);
    for(int i = 0; i < (to_ith_link+1); i++)
    {
        DQ x_i_to_i_plus_one = _dh2dq(q_vec(i),i);
        x_backward = conj(x_i_to_i_plus_one)*x_backward;
        DQ x_ith_dot  = x_forward*_dh2dq_dot(q_vec(i),i)*x_backward;
        x_forward  = x_forward*x_i_to_i_plus_one;
        J.col(i) = vec8(x_ith_dot);
    }
    return J;
}


}
