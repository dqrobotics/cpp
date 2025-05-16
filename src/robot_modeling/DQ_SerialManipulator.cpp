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

3. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
    - Added the joint_types member, and the following methods:
      _check_joint_types(), and {set,get}_joint_{type, types}.
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

/**
 * @brief DQ_SerialManipulator::_check_joint_types throws an exception if the joint types are
 *                           different from the supported joints.
 */
void DQ_SerialManipulator::_check_joint_types() const
{
    std::vector<DQ_JointType> types = get_joint_types();
    std::vector<DQ_JointType> supported_types = get_supported_joint_types();
    std::string msg = "Unsupported joint types. Use valid joint types: ";
    std::string msg_type;
    std::string ps;
    size_t k = supported_types.size();
    size_t n = types.size();
    for (size_t i=0;i<k;i++)
    {
        msg_type = std::string("DQ_JointType::"+supported_types.at(i).ToString());
        if (i==k-1)
            ps = std::string(". ");
        else
            ps = std::string(", ");

        msg += msg_type + ps;
    }


    for (size_t i=0;i<n;i++)
    {
        bool match = false;
        for (size_t j=0;j<k;j++)
        {
            if (types.at(i) == supported_types.at(j))
            {
                match = true;
                break;
            }
        }
        if (match == false)
            throw std::runtime_error(msg);
    }
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

/**
 * @brief DQ_SerialManipulator::get_joint_type returns the joint type of the ith joint.
 * @param ith_joint The index to a joint.
 * @return The desired ith joint type.
 */
DQ_JointType DQ_SerialManipulator::get_joint_type(const int &ith_joint) const
{
    _check_to_ith_link(ith_joint);
    return joint_types_.at(ith_joint);
}

/**
 * @brief DQ_SerialManipulator::get_joint_types returns a vector containing the joint types.
 * @return The desired joint types.
 */
std::vector<DQ_JointType> DQ_SerialManipulator::get_joint_types() const
{
    return joint_types_;
}

/**
 * @brief DQ_SerialManipulator::set_joint_type sets the joint type of the ith joint
 * @param joint_type The joint_type.
 * @param ith_joint The index to a joint.
 */
void DQ_SerialManipulator::set_joint_type(const DQ_JointType &joint_type, const int &ith_joint)
{
    _check_to_ith_link(ith_joint);
    joint_types_.at(ith_joint) = joint_type;
    _check_joint_types();
}

/**
 * @brief DQ_SerialManipulator::set_joint_types sets the joint types.
 * @param joint_types A vector containing the joint types.
 */
void DQ_SerialManipulator::set_joint_types(const std::vector<DQ_JointType> &joint_types)
{
    joint_types_ = joint_types;
    _check_joint_types();
}

/**
 * @brief DQ_SerialManipulator::set_joint_types sets the joint types.
 * @param joint_types A vector containing the joint types.
 */
void DQ_SerialManipulator::set_joint_types(const VectorXd &joint_types)
{
    for (int i=0;i<joint_types.size();i++)
        joint_types_.push_back(DQ_JointType(joint_types(i)));
    _check_joint_types();
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

