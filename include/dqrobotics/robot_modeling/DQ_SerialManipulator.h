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
#include <vector>

namespace DQ_robotics
{

/**
 * @brief Abstract class that defines serial manipulators.
 *
 * DQ_SerialManipulator extends DQ_Kinematics with the common operations of
 * fixed-base and mobile-base serial chains. Subclasses implement the raw
 * forward kinematics and raw Jacobians for a specific parameterization,
 * while this class applies the reference frame and optional end-effector
 * rigid transformation.
 *
 * @note The raw_* methods with = 0 are pure virtual and must be implemented
 * by subclasses. The overloads without an explicit link index are concrete
 * wrappers that default to the last link.
 *
 * @see DQ_Kinematics, DQ_SerialManipulatorDH, DQ_SerialManipulatorMDH
 */
class DQ_SerialManipulator: public DQ_Kinematics
{
protected:
    /** @brief Constant rigid transformation from the last link to the end effector. */
    DQ curr_effector_;
    /** @brief Actuation type associated with each joint of the chain. */
    std::vector<DQ_JointType> joint_types_;
    /**
     * @brief Constructs a serial manipulator with the given number of degrees of freedom.
     *
     * @param dofs Dimension of the configuration space.
     */
    DQ_SerialManipulator(const int& dofs);
    /**
     * @brief Verifies whether the stored joint types are supported by the subclass.
     *
     * @throws std::runtime_error If at least one stored joint type is not supported.
     */
    void _check_joint_types() const;
public:
    /**
     * @brief Returns the current end-effector rigid transformation.
     *
     * @return The constant rigid transformation appended to the last link.
     */
    DQ get_effector() const;
    /**
     * @brief Sets the current end-effector rigid transformation.
     *
     * @param new_effector Constant rigid transformation from the last link to the tool frame.
     * @return The stored end-effector transformation.
     */
    DQ set_effector(const DQ& new_effector);

    /** @brief Returns the lower joint-position limits. */
    VectorXd get_lower_q_limit() const;
    /**
     * @brief Sets the lower joint-position limits.
     *
     * @param lower_q_limit Vector containing the lower position limits.
     */
    void     set_lower_q_limit(const VectorXd& lower_q_limit);
    /** @brief Returns the lower joint-velocity limits. */
    VectorXd get_lower_q_dot_limit() const;
    /**
     * @brief Sets the lower joint-velocity limits.
     *
     * @param lower_q_dot_limit Vector containing the lower velocity limits.
     */
    void     set_lower_q_dot_limit(const VectorXd &lower_q_dot_limit);
    /** @brief Returns the upper joint-position limits. */
    VectorXd get_upper_q_limit() const;
    /**
     * @brief Sets the upper joint-position limits.
     *
     * @param upper_q_limit Vector containing the upper position limits.
     */
    void     set_upper_q_limit(const VectorXd& upper_q_limit);
    /** @brief Returns the upper joint-velocity limits. */
    VectorXd get_upper_q_dot_limit() const;
    /**
     * @brief Sets the upper joint-velocity limits.
     *
     * @param upper_q_dot_limit Vector containing the upper velocity limits.
     */
    void     set_upper_q_dot_limit(const VectorXd &upper_q_dot_limit);

    /**
     * @brief Returns the actuation type of a given joint.
     *
     * @param ith_joint Joint index.
     * @return The actuation type of the selected joint.
     */
    DQ_JointType                get_joint_type(const int& ith_joint) const;
    /**
     * @brief Returns the actuation types of all joints.
     *
     * @return Vector containing the actuation type of each joint.
     */
    std::vector<DQ_JointType>   get_joint_types() const;
    /**
     * @brief Sets the actuation type of a given joint.
     *
     * @param joint_type Joint type to be assigned.
     * @param ith_joint Joint index.
     * @throws std::runtime_error If the resulting list of joint types is not supported.
     */
    void  set_joint_type(const DQ_JointType& joint_type, const int& ith_joint);
    /**
     * @brief Sets the actuation types of all joints.
     *
     * @param joint_types Vector containing the desired joint types.
     * @throws std::runtime_error If at least one joint type is not supported.
     */
    void  set_joint_types(const std::vector<DQ_JointType>& joint_types);
    /**
     * @brief Sets the actuation types of all joints from numeric values.
     *
     * Each entry is converted to a DQ_JointType according to the integer-based
     * constructor of DQ_JointType.
     *
     * @param joint_types Numeric vector whose entries encode the desired joint types.
     * @throws std::runtime_error If at least one joint type is invalid or not supported.
     */
    void  set_joint_types(const VectorXd& joint_types);

    //Virtual
    /**
     * @brief Computes the raw pose Jacobian up to the last link.
     *
     * This concrete overload delegates to raw_pose_jacobian(q_vec, get_dim_configuration_space() - 1).
     *
     * @note This function does not take into account any reference-frame or end-effector
     * displacements, and is intended mostly for internal use by DQ_Kinematics and its
     * subclasses. Most users should call pose_jacobian() instead, which applies those
     * transformations.
     *
     * @param q_vec Vector containing the robot joint configurations.
     * @return The raw pose Jacobian, without reference-frame or end-effector transformations.
     */
    virtual MatrixXd raw_pose_jacobian(const VectorXd& q_vec) const;
    /**
     * @brief Computes the time derivative of the raw pose Jacobian up to the last link.
     *
     * This concrete overload delegates to raw_pose_jacobian_derivative(q, q_dot, get_dim_configuration_space() - 1).
     *
     * @note This function does not take into account any reference-frame or end-effector
     * displacements, and is intended mostly for internal use by DQ_Kinematics and its
     * subclasses. Most users should call pose_jacobian_derivative() instead, which applies
     * those transformations.
     *
     * @param q Vector containing the robot joint configurations.
     * @param q_dot Vector containing the robot joint velocities.
     * @return The derivative of the raw pose Jacobian.
     */
    virtual MatrixXd raw_pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot) const;
    /**
     * @brief Computes the raw forward kinematics up to the last link.
     *
     * This concrete overload delegates to raw_fkm(q_vec, get_dim_configuration_space() - 1).
     *
     * @note This function does not take into account any reference-frame or end-effector
     * displacements. It is an auxiliary function intended mostly for internal use, chiefly
     * by the pose-Jacobian computations. Most users should call fkm() instead.
     *
     * @param q_vec Vector containing the robot joint configurations.
     * @return The pose of the last link before applying the reference frame and the end effector.
     */
    virtual DQ raw_fkm(const VectorXd& q_vec) const;

    //Pure virtual
    /**
     * @brief Computes the raw pose Jacobian up to a given link.
     *
     * This is a pure virtual interface contract and must be implemented by subclasses.
     *
     * @note This function does not take into account any reference-frame or end-effector
     * displacements, and is intended mostly for internal use by DQ_Kinematics and its
     * subclasses. Most users should call pose_jacobian() instead, which applies those
     * transformations.
     *
     * @param q_vec Vector containing the robot joint configurations.
     * @param to_ith_link Index of the last link to be accounted for in the computation.
     * @return The raw pose Jacobian, without reference-frame or end-effector transformations.
     */
    virtual MatrixXd raw_pose_jacobian(const VectorXd& q_vec, const int& to_ith_link) const = 0;
    /**
     * @brief Computes the time derivative of the raw pose Jacobian up to a given link.
     *
     * This is a pure virtual interface contract and must be implemented by subclasses.
     *
     * @note This function does not take into account any reference-frame or end-effector
     * displacements, and is intended mostly for internal use by DQ_Kinematics and its
     * subclasses. Most users should call pose_jacobian_derivative() instead, which applies
     * those transformations.
     *
     * @param q Vector containing the robot joint configurations.
     * @param q_dot Vector containing the robot joint velocities.
     * @param to_ith_link Index of the last link to be accounted for in the computation.
     * @return The derivative of the raw pose Jacobian.
     */
    virtual MatrixXd raw_pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot, const int& to_ith_link) const = 0;
    /**
     * @brief Computes the raw forward kinematics up to a given link.
     *
     * This is a pure virtual interface contract and must be implemented by subclasses.
     *
     * @note This function does not take into account any reference-frame or end-effector
     * displacements. It is an auxiliary function intended mostly for internal use, chiefly
     * by the pose-Jacobian computations. Most users should call fkm() instead.
     *
     * @param q_vec Vector containing the robot joint configurations.
     * @param to_ith_link Index of the last link to be accounted for in the computation.
     * @return The raw pose of the ith link, without reference-frame or end-effector transformations.
     */
    virtual DQ raw_fkm(const VectorXd& q_vec, const int& to_ith_link) const = 0;
    /**
     * @brief Returns the joint types supported by the subclass.
     *
     * This is a pure virtual interface contract and must be implemented by subclasses.
     *
     * @return Vector containing the supported joint types.
     */
    virtual std::vector<DQ_JointType> get_supported_joint_types() const = 0;

    //Overrides from DQ_Kinematics
    /**
     * @brief Computes the forward kinematics of the end effector.
     *
     * This concrete override applies the reference frame and the stored
     * end-effector rigid transformation.
     *
     * @param q_vec Vector containing the robot joint configurations.
     * @return The pose of the end effector as a unit dual quaternion.
     */
    virtual DQ fkm(const VectorXd& q_vec) const override; //Override from DQ_Kinematics
    /**
     * @brief Computes the forward kinematics up to a given link.
     *
     * The returned pose includes the reference frame. The stored end-effector
     * transformation is applied only when @p to_ith_link is the last link.
     *
     * @param q_vec Vector containing the robot joint configurations.
     * @param to_ith_link Index of the last link to be accounted for in the computation.
     * @return The pose of the ith link as a unit dual quaternion.
     */
    virtual DQ fkm(const VectorXd& q_vec, const int& to_ith_link) const override; //Override from DQ_Kinematics

    /**
     * @brief Returns the dimension of the configuration space.
     *
     * @return Number of generalized coordinates of the serial manipulator.
     */
    virtual int get_dim_configuration_space() const override; //Override from DQ_Kinematics

    /**
     * @brief Computes the pose Jacobian up to a given link.
     *
     * The returned Jacobian includes the reference frame. The stored end-effector
     * transformation is applied only when @p to_ith_link is the last link.
     *
     * @param q_vec Vector containing the robot joint configurations.
     * @param to_ith_link Index of the last link to be accounted for in the computation.
     * @return The pose Jacobian that satisfies vec8(pose_dot) = J * q_dot.
     */
    virtual MatrixXd pose_jacobian(const VectorXd& q_vec, const int& to_ith_link) const override; //Override from DQ_Kinematics
    /**
     * @brief Computes the pose Jacobian of the end effector.
     *
     * @param q_vec Vector containing the robot joint configurations.
     * @return The pose Jacobian that satisfies vec8(pose_dot) = J * q_dot.
     */
    virtual MatrixXd pose_jacobian(const VectorXd& q_vec) const override; //Override from DQ_Kinematics
    /**
     * @brief Computes the time derivative of the pose Jacobian up to a given link.
     *
     * The returned Jacobian derivative includes the reference frame. The stored
     * end-effector transformation is applied only when @p to_ith_link is the last link.
     *
     * @param q Vector containing the robot joint configurations.
     * @param q_dot Vector containing the robot joint velocities.
     * @param to_ith_link Index of the last link to be accounted for in the computation.
     * @return The derivative of the pose Jacobian.
     */
    virtual MatrixXd pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot, const int& to_ith_link) const override; //Override from DQ_Kinematics
    /**
     * @brief Computes the time derivative of the pose Jacobian of the end effector.
     *
     * @param q Vector containing the robot joint configurations.
     * @param q_dot Vector containing the robot joint velocities.
     * @return The derivative of the pose Jacobian.
     */
    virtual MatrixXd pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot) const override; //Override from DQ_Kinematics

};

}
