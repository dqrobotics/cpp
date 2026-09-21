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
1. Murilo M. Marinho (murilomarinho@ieee.org)
    - Responsible for the original implementation.

2. Juan Jose Quiroz Omana (juanjqogm@gmail.com)
    - Added methods to get and set the DH parameters.
    - Added the get_supported_joint_types() method.
*/


#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include <dqrobotics/robot_modeling/DQ_ParameterDH.h>

namespace DQ_robotics
{

/**
 * @brief Concrete serial manipulator based on the standard Denavit-Hartenberg convention.
 *
 * The constructor expects a 5 x n matrix whose rows store theta, d, a, alpha,
 * and the joint type of each link. Revolute joints use the theta row as joint
 * offset, whereas prismatic joints use the d row as joint offset.
 *
 * @see DQ_SerialManipulator, DQ_SerialManipulatorMDH, DQ_ParameterDH
 */
class DQ_SerialManipulatorDH: public DQ_SerialManipulator
{
protected:
    /** @brief Matrix storing theta, d, a, alpha, and joint-type rows. */
    MatrixXd    dh_matrix_;

    /**
     * @brief Returns the dual quaternion term used in the pose-Jacobian derivative recursion.
     *
     * @param ith Link index.
     * @return The dual quaternion term associated with the ith joint under the standard DH convention.
     */
    DQ _get_w(const int& ith) const;
    /**
     * @brief Converts the ith standard DH link transform into a dual quaternion.
     *
     * @param q Joint value associated with the ith joint.
     * @param ith Link index.
     * @return The dual quaternion representing the ith link transform.
     */
    DQ _dh2dq(const double& q, const int& ith) const;
public:
    /**
     * @brief Returns one row of the stored DH matrix.
     *
     * @param parameter_type DH parameter to be retrieved.
     * @return Vector containing the selected parameter for all joints.
     * @throws std::runtime_error If @p parameter_type is not supported.
     */
    VectorXd get_parameters(const DQ_ParameterDH& parameter_type) const;
    /**
     * @brief Returns a DH parameter of a specific joint.
     *
     * @param parameter_type DH parameter to be retrieved.
     * @param to_ith_link Joint index.
     * @return Value of the selected parameter at the requested joint.
     * @throws std::runtime_error If @p parameter_type is not supported.
     */
    double   get_parameter(const DQ_ParameterDH& parameter_type,
                           const int& to_ith_link) const;
    /**
     * @brief Replaces one row of the stored DH matrix.
     *
     * @param parameter_type DH parameter to be updated.
     * @param vector_parameters Vector containing the new values for all joints.
     */
    void set_parameters(const DQ_ParameterDH& parameter_type,
                        const VectorXd& vector_parameters);
    /**
     * @brief Sets a DH parameter of a specific joint.
     *
     * @param parameter_type DH parameter to be updated.
     * @param to_ith_link Joint index.
     * @param parameter New parameter value.
     */
    void set_parameter(const DQ_ParameterDH& parameter_type,
                       const int& to_ith_link,
                       const double& parameter);

    /**
     * @brief Returns the joint types supported by the standard DH implementation.
     *
     * @return Vector containing REVOLUTE and PRISMATIC.
     */
    std::vector<DQ_JointType> get_supported_joint_types()const override;

    // Deprecated on 22.04, will be removed on the next release.
    /** @brief Deprecated joint-type constants kept for backward compatibility. */
    enum [[deprecated("Use DQ_JointType instead.")]] JOINT_TYPES{ JOINT_ROTATIONAL=0, /**< Revolute joint. */ JOINT_PRISMATIC /**< Prismatic joint. */ };

    /**
     * @brief Returns the theta row of the stored DH matrix.
     * @return Vector containing all theta parameters.
     * @deprecated Use get_parameters(DQ_ParameterDH::THETA) instead.
     */
    [[deprecated("Use get_parameters(DQ_ParameterDH::THETA) instead.")]] VectorXd get_thetas() const;
    /**
     * @brief Returns the d row of the stored DH matrix.
     * @return Vector containing all d parameters.
     * @deprecated Use get_parameters(DQ_ParameterDH::D) instead.
     */
    [[deprecated("Use get_parameters(DQ_ParameterDH::D) instead.")]]     VectorXd get_ds() const;
    /**
     * @brief Returns the a row of the stored DH matrix.
     * @return Vector containing all a parameters.
     * @deprecated Use get_parameters(DQ_ParameterDH::A) instead.
     */
    [[deprecated("Use get_parameters(DQ_ParameterDH::A) instead.")]]     VectorXd get_as() const;
    /**
     * @brief Returns the alpha row of the stored DH matrix.
     * @return Vector containing all alpha parameters.
     * @deprecated Use get_parameters(DQ_ParameterDH::ALPHA) instead.
     */
    [[deprecated("Use get_parameters(DQ_ParameterDH::ALPHA) instead.")]] VectorXd get_alphas() const;
    /**
     * @brief Returns the joint-type row of the stored DH matrix.
     * @return Vector containing the encoded joint types.
     * @deprecated Use get_joint_types() instead.
     */
    [[deprecated("Use get_joint_types() instead.")]]                     VectorXd get_types() const;

    /** @brief Deleted default constructor. */
    DQ_SerialManipulatorDH()=delete;
    /**
     * @brief Constructs a serial manipulator from a standard DH matrix.
     *
     * @param dh_matrix 5 x n matrix containing theta, d, a, alpha, and joint-type rows.
     * @throws std::range_error If @p dh_matrix does not have exactly 5 rows.
     */
    DQ_SerialManipulatorDH(const MatrixXd& dh_matrix);

    /** @brief Exposes the overload that computes the raw pose Jacobian up to the last link. */
    using DQ_SerialManipulator::raw_pose_jacobian;
    /** @brief Exposes the overload that computes the raw pose-Jacobian derivative up to the last link. */
    using DQ_SerialManipulator::raw_pose_jacobian_derivative;
    /** @brief Exposes the overload that computes the raw forward kinematics up to the last link. */
    using DQ_SerialManipulator::raw_fkm;

    /**
     * @brief Computes the raw pose Jacobian under the standard DH convention.
     *
     * @param q_vec Vector containing the robot joint configurations.
     * @param to_ith_link Index of the last link to be accounted for in the computation.
     * @return The raw pose Jacobian up to the requested link.
     */
    MatrixXd raw_pose_jacobian(const VectorXd& q_vec, const int& to_ith_link) const override;
    /**
     * @brief Computes the time derivative of the raw pose Jacobian under the standard DH convention.
     *
     * @param q Vector containing the robot joint configurations.
     * @param q_dot Vector containing the robot joint velocities.
     * @param to_ith_link Index of the last link to be accounted for in the computation.
     * @return The derivative of the raw pose Jacobian up to the requested link.
     */
    MatrixXd raw_pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot, const int& to_ith_link) const override;
    /**
     * @brief Computes the raw forward kinematics under the standard DH convention.
     *
     * @param q_vec Vector containing the robot joint configurations.
     * @param to_ith_link Index of the last link to be accounted for in the computation.
     * @return The raw pose of the requested link.
     */
    DQ raw_fkm(const VectorXd &q_vec, const int &to_ith_link) const override;
};

}
