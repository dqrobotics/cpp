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
1. Murilo M. Marinho        (murilomarinho@ieee.org)
2. Mateus Rodrigues Martins (martinsrmateus@gmail.com)

3. Juan Jose Quiroz Omana (juanjqogm@gmail.com)
    - Added the enum class DQ_ParameterDH
*/

#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include <math.h>       //library for math functions
#include <stdexcept>    //For range_error
#include <string>

namespace DQ_robotics
{

DQ_SerialManipulator::DQ_SerialManipulator(const int &dim_configuration_space):
    DQ_Kinematics()
{
    curr_effector_ = DQ(1);
    lower_q_limit_.resize(dim_configuration_space);
    upper_q_limit_.resize(dim_configuration_space);
    lower_q_dot_limit_.resize(dim_configuration_space);
    upper_q_dot_limit_.resize(dim_configuration_space);
    dim_configuration_space_ = dim_configuration_space;
}


int  DQ_SerialManipulator::get_dim_configuration_space() const
{
    return dim_configuration_space_;
}

DQ  DQ_SerialManipulator::get_effector() const
{
    return curr_effector_;
}

DQ  DQ_SerialManipulator::set_effector( const DQ& new_effector)
{
    curr_effector_ = new_effector;
    return curr_effector_;
}

VectorXd DQ_SerialManipulator::get_lower_q_limit() const
{
    return lower_q_limit_;
}

void DQ_SerialManipulator::set_lower_q_limit(const VectorXd &lower_q_limit)
{
    lower_q_limit_ = lower_q_limit;
}

VectorXd DQ_SerialManipulator::get_lower_q_dot_limit() const
{
    return lower_q_dot_limit_;
}

void DQ_SerialManipulator::set_lower_q_dot_limit(const VectorXd &lower_q_dot_limit)
{
    lower_q_dot_limit_ = lower_q_dot_limit;
}

VectorXd DQ_SerialManipulator::get_upper_q_limit() const
{
    return upper_q_limit_;
}

void DQ_SerialManipulator::set_upper_q_limit(const VectorXd &upper_q_limit)
{
    upper_q_limit_ = upper_q_limit;
}

VectorXd DQ_SerialManipulator::get_upper_q_dot_limit() const
{
    return upper_q_dot_limit_;
}

void DQ_SerialManipulator::set_upper_q_dot_limit(const VectorXd &upper_q_dot_limit)
{
    upper_q_dot_limit_ = upper_q_dot_limit;
}

DQ  DQ_SerialManipulator::raw_fkm(const VectorXd& q_vec) const
{
    _check_q_vec(q_vec);

    return raw_fkm(q_vec, get_dim_configuration_space() - 1);
}

DQ  DQ_SerialManipulator::fkm(const VectorXd& q_vec) const
{
    return fkm(q_vec, get_dim_configuration_space()-1);
}

DQ  DQ_SerialManipulator::fkm(const VectorXd& q_vec, const int& to_ith_link) const
{
    _check_q_vec(q_vec);
    _check_to_ith_link(to_ith_link);

    DQ q = reference_frame_ * raw_fkm(q_vec, to_ith_link); //Take the base into account

    if(to_ith_link == get_dim_configuration_space() - 1)
        q = q * curr_effector_; //Take into account the end effector

    return q;
}

MatrixXd DQ_SerialManipulator::raw_pose_jacobian(const VectorXd &q_vec) const
{
    _check_q_vec(q_vec);

    return raw_pose_jacobian(q_vec, get_dim_configuration_space()-1);
}


MatrixXd  DQ_SerialManipulator::pose_jacobian(const VectorXd& q_vec, const int &to_ith_link) const
{
    _check_q_vec(q_vec);
    _check_to_ith_link(to_ith_link);

    MatrixXd J = raw_pose_jacobian(q_vec,to_ith_link);

    if(to_ith_link==get_dim_configuration_space()-1)
    {
        J = hamiplus8(reference_frame_)*haminus8(curr_effector_)*J;
    }
    else
    {
        J = hamiplus8(reference_frame_)*J;
    }

    return J;
}

MatrixXd DQ_SerialManipulator::pose_jacobian(const VectorXd &q_vec) const
{
    _check_q_vec(q_vec);

    return DQ_Kinematics::pose_jacobian(q_vec);
}


/**
 * @brief This method returns the time derivative of the pose Jacobian.
 *        The base displacement and the effector are not taken into account.
 * @param q. VectorXd representing the robot joint configuration.
 * @param q_dot. VectorXd representing the robot joint velocities.
 * @returns a MatrixXd representing the desired Jacobian derivative.
 *
 */
MatrixXd DQ_SerialManipulator::raw_pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot) const
{
    _check_q_vec(q);
    _check_q_vec(q_dot);
    return raw_pose_jacobian_derivative(q, q_dot, get_dim_configuration_space()-1);
}


/**
 * @brief This method returns the first to_ith_link columns of the time derivative of the pose Jacobian.
 * @param q. VectorXd representing the robot joint configuration.
 * @param q_dot. VectorXd representing the robot joint velocities.
 * @param to_ith_link. The index to a link. This defines until which link the pose_jacobian_derivative
 *                     will be calculated.
 * @returns a MatrixXd representing the first to_ith_link columns of the desired Jacobian derivative.
 *
 */
MatrixXd  DQ_SerialManipulator::pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot, const int &to_ith_link) const
{
    _check_q_vec(q);
    _check_q_vec(q_dot);
    _check_to_ith_link(to_ith_link);
    MatrixXd J_dot = raw_pose_jacobian_derivative(q, q_dot, to_ith_link);

    if(to_ith_link==get_dim_configuration_space()-1)
    {
        J_dot = hamiplus8(reference_frame_)*haminus8(curr_effector_)*J_dot;
    }
    else
    {
        J_dot = hamiplus8(reference_frame_)*J_dot;
    }
    return J_dot;
}

/**
 * @brief This method returns time derivative of the pose Jacobian.
 * @param q. VectorXd representing the robot joint configuration.
 * @param q_dot. VectorXd representing the robot joint velocities.
 * @returns a MatrixXd representing the desired Jacobian derivative.
 *
 */
MatrixXd DQ_SerialManipulator::pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot) const
{
    _check_q_vec(q);
    _check_q_vec(q_dot);
    return DQ_Kinematics::pose_jacobian_derivative(q, q_dot);
}

}//namespace DQ_robotics

