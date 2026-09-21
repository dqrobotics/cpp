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

2. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
    - Added the get_supported_joint_types() method.
*/

#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>

namespace DQ_robotics
{

/**
 * @brief Concrete serial manipulator that uses the DENSO kinematic convention.
 *
 * The constructor expects a 6 x n matrix whose rows store the convention
 * parameters a, b, d, alpha, beta, and gamma for each link. The current
 * implementation supports revolute joints only.
 *
 * @see DQ_SerialManipulator, DQ_JointType
 */
class DQ_SerialManipulatorDenso: public DQ_SerialManipulator
{
protected:
    /** @brief Matrix storing a, b, d, alpha, beta, and gamma rows. */
    MatrixXd    denso_matrix_;

    /**
     * @brief Converts the ith DENSO link transform into a dual quaternion.
     *
     * @param q Joint value associated with the ith joint.
     * @param ith Link index.
     * @return The dual quaternion representing the ith link transform.
     */
    DQ _denso2dh(const double& q, const int& ith) const;
public:
    /**
     * @brief Returns the joint types supported by the DENSO implementation.
     *
     * @return Vector containing only REVOLUTE.
     */
    std::vector<DQ_JointType> get_supported_joint_types() const override;

    // Deprecated on 22.04, will be removed on the next release.
    /**
     * @brief Returns the a row of the stored DENSO matrix.
     * @return Vector containing all a parameters.
     * @deprecated This accessor is kept for backward compatibility.
     */
    [[deprecated("Use ? instead.")]] VectorXd get_as() const;
    /**
     * @brief Returns the b row of the stored DENSO matrix.
     * @return Vector containing all b parameters.
     * @deprecated This accessor is kept for backward compatibility.
     */
    [[deprecated("Use ? instead.")]] VectorXd get_bs() const;
    /**
     * @brief Returns the d row of the stored DENSO matrix.
     * @return Vector containing all d parameters.
     * @deprecated This accessor is kept for backward compatibility.
     */
    [[deprecated("Use ? instead.")]] VectorXd get_ds() const;
    /**
     * @brief Returns the alpha row of the stored DENSO matrix.
     * @return Vector containing all alpha parameters.
     * @deprecated This accessor is kept for backward compatibility.
     */
    [[deprecated("Use ? instead.")]] VectorXd get_alphas() const;
    /**
     * @brief Returns the beta row of the stored DENSO matrix.
     * @return Vector containing all beta parameters.
     * @deprecated This accessor is kept for backward compatibility.
     */
    [[deprecated("Use ? instead.")]] VectorXd get_betas() const;
    /**
     * @brief Returns the gamma row of the stored DENSO matrix.
     * @return Vector containing all gamma parameters.
     * @deprecated This accessor is kept for backward compatibility.
     */
    [[deprecated("Use ? instead.")]] VectorXd get_gammas() const;

    /** @brief Deleted default constructor. */
    DQ_SerialManipulatorDenso()=delete;
    /**
     * @brief Constructs a serial manipulator from a DENSO-parameter matrix.
     *
     * @param denso_matrix 6 x n matrix containing a, b, d, alpha, beta, and gamma rows.
     * @throws std::range_error If @p denso_matrix does not have exactly 6 rows.
     */
    DQ_SerialManipulatorDenso(const MatrixXd& denso_matrix);

    /** @brief Exposes the overload that computes the raw pose Jacobian up to the last link. */
    using DQ_SerialManipulator::raw_pose_jacobian;
    /** @brief Exposes the overload that computes the raw pose-Jacobian derivative up to the last link. */
    using DQ_SerialManipulator::raw_pose_jacobian_derivative;
    /** @brief Exposes the overload that computes the raw forward kinematics up to the last link. */
    using DQ_SerialManipulator::raw_fkm;

    /**
     * @brief Computes the raw pose Jacobian under the DENSO convention.
     *
     * @param q_vec Vector containing the robot joint configurations.
     * @param to_ith_link Index of the last link to be accounted for in the computation.
     * @return The raw pose Jacobian up to the requested link.
     */
    MatrixXd raw_pose_jacobian(const VectorXd& q_vec, const int& to_ith_link) const override;
    /**
     * @brief Computes the time derivative of the raw pose Jacobian under the DENSO convention.
     *
     * @param q Vector containing the robot joint configurations.
     * @param q_dot Vector containing the robot joint velocities.
     * @param to_ith_link Index of the last link to be accounted for in the computation.
     * @return The derivative of the raw pose Jacobian up to the requested link.
     */
    MatrixXd raw_pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot, const int& to_ith_link) const override;
    /**
     * @brief Computes the raw forward kinematics under the DENSO convention.
     *
     * @param q_vec Vector containing the robot joint configurations.
     * @param to_ith_link Index of the last link to be accounted for in the computation.
     * @return The raw pose of the requested link.
     */
    DQ raw_fkm(const VectorXd &q_vec, const int &to_ith_link) const override;
};

}
