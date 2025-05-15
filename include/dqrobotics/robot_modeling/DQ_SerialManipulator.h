#pragma once
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

#include <dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <dqrobotics/robot_modeling/DQ_JointType.h>

namespace DQ_robotics
{

class DQ_SerialManipulator: public DQ_Kinematics
{
protected:
    DQ curr_effector_;
    std::vector<DQ_JointType> joint_types_;
    DQ_SerialManipulator(const int& dofs);
    void _check_joint_types() const;
public:
    DQ get_effector() const;
    DQ set_effector(const DQ& new_effector);

    VectorXd get_lower_q_limit() const;
    void     set_lower_q_limit(const VectorXd& lower_q_limit);
    VectorXd get_lower_q_dot_limit() const;
    void     set_lower_q_dot_limit(const VectorXd &lower_q_dot_limit);
    VectorXd get_upper_q_limit() const;
    void     set_upper_q_limit(const VectorXd& upper_q_limit);
    VectorXd get_upper_q_dot_limit() const;
    void     set_upper_q_dot_limit(const VectorXd &upper_q_dot_limit);

    DQ_JointType                get_joint_type(const int& ith_joint) const;
    std::vector<DQ_JointType>   get_joint_types() const;
    void  set_joint_type(const DQ_JointType& joint_type, const int& ith_joint);
    void  set_joint_types(const std::vector<DQ_JointType>& joint_types);
    void  set_joint_types(const VectorXd& joint_types);

    //Virtual
    virtual MatrixXd raw_pose_jacobian(const VectorXd& q_vec) const;
    virtual MatrixXd raw_pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot) const;
    virtual DQ raw_fkm(const VectorXd& q_vec) const;

    //Pure virtual
    virtual MatrixXd raw_pose_jacobian(const VectorXd& q_vec, const int& to_ith_link) const = 0;
    virtual MatrixXd raw_pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot, const int& to_ith_link) const = 0;
    virtual DQ raw_fkm(const VectorXd& q_vec, const int& to_ith_link) const = 0;
    virtual std::vector<DQ_JointType> get_supported_joint_types() const = 0;

    //Overrides from DQ_Kinematics
    virtual DQ fkm(const VectorXd& q_vec) const override; //Override from DQ_Kinematics
    virtual DQ fkm(const VectorXd& q_vec, const int& to_ith_link) const override; //Override from DQ_Kinematics

    virtual int get_dim_configuration_space() const override; //Override from DQ_Kinematics

    virtual MatrixXd pose_jacobian(const VectorXd& q_vec, const int& to_ith_link) const override; //Override from DQ_Kinematics
    virtual MatrixXd pose_jacobian(const VectorXd& q_vec) const override; //Override from DQ_Kinematics
    virtual MatrixXd pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot, const int& to_ith_link) const override; //Override from DQ_Kinematics
    virtual MatrixXd pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot) const override; //Override from DQ_Kinematics

};

}
