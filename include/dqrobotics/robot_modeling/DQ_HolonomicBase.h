/**
(C) Copyright 2019 DQ Robotics Developers

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
- Murilo M. Marinho (murilomarinho@ieee.org)
*/

#ifndef DQ_ROBOTICS_ROBOT_MODELING_DQ_HOLONOMICBASE
#define DQ_ROBOTICS_ROBOT_MODELING_DQ_HOLONOMICBASE

#include<dqrobotics/DQ.h>
#include<dqrobotics/robot_modeling/DQ_MobileBase.h>

namespace DQ_robotics
{


/**
 * @brief Basic implementation of a holonomic mobile base.
 *
 * The configuration vector is q = [x, y, phi]^T, where x and y describe the
 * planar position and phi is the planar orientation. The concrete methods in
 * this class compute the raw pose and raw Jacobians of that planar motion and
 * optionally account for an additional frame displacement.
 *
 * @see DQ_MobileBase, DQ_DifferentialDriveRobot
 */
class DQ_HolonomicBase: public DQ_MobileBase
{
public:
    /**
     * @brief Constructs a holonomic base.
     *
     * The configuration space has dimension three.
     */
    DQ_HolonomicBase();

    //Virtual method overloads (DQ_Kinematics)
    /**
     * @brief Computes the mobile-base pose while considering the frame displacement.
     *
     * @param q Configuration vector [x, y, phi]^T.
     * @return The mobile-base pose as a unit dual quaternion.
     */
    virtual DQ fkm(const VectorXd& q) const override;
    /**
     * @brief Computes the mobile-base pose while considering the frame displacement.
     *
     * This overload exists for compatibility with the generic kinematic interface.
     * The current implementation accepts only @p to_ith_link = 2.
     *
     * @param q Configuration vector [x, y, phi]^T.
     * @param to_ith_link Link index.
     * @return The mobile-base pose as a unit dual quaternion.
     * @throws std::runtime_error If @p to_ith_link is different from 2.
     */
    virtual DQ fkm(const VectorXd& q, const int& to_ith_link) const override;
    /**
     * @brief Computes the pose Jacobian while considering the frame displacement.
     *
     * @param q Configuration vector [x, y, phi]^T.
     * @param to_link Column index of the partial Jacobian to be returned.
     * @return The pose Jacobian up to the requested column.
     * @throws std::runtime_error If @p to_link is outside the interval [0, 2].
     */
    virtual MatrixXd pose_jacobian(const VectorXd& q, const int& to_link) const override;
    /**
     * @brief Computes the full pose Jacobian while considering the frame displacement.
     *
     * @param q Configuration vector [x, y, phi]^T.
     * @return The full pose Jacobian.
     */
    virtual MatrixXd pose_jacobian(const VectorXd& q) const override;
    /**
     * @brief Computes the time derivative of the pose Jacobian while considering the frame displacement.
     *
     * @param q Configuration vector [x, y, phi]^T.
     * @param q_dot Configuration-velocity vector [x_dot, y_dot, phi_dot]^T.
     * @param to_link Column index of the partial Jacobian derivative to be returned.
     * @return The pose-Jacobian derivative up to the requested column.
     * @throws std::runtime_error If @p to_link is outside the interval [0, 2].
     */
    virtual MatrixXd pose_jacobian_derivative(const VectorXd& q,
                                              const VectorXd& q_dot, const int& to_link) const override;
    /**
     * @brief Computes the full time derivative of the pose Jacobian.
     *
     * @param q Configuration vector [x, y, phi]^T.
     * @param q_dot Configuration-velocity vector [x_dot, y_dot, phi_dot]^T.
     * @return The full pose-Jacobian derivative.
     */
    virtual MatrixXd pose_jacobian_derivative(const VectorXd& q,
                                              const VectorXd& q_dot) const override;

    /**
     * @brief Computes the planar mobile-base pose without considering the frame displacement.
     *
     * @param q Configuration vector [x, y, phi]^T.
     * @return The raw mobile-base pose as a unit dual quaternion.
     */
    DQ raw_fkm(const VectorXd& q) const;
    /**
     * @brief Computes the raw pose Jacobian of the planar mobile base.
     *
     * @param q Configuration vector [x, y, phi]^T.
     * @param to_link Column index of the partial Jacobian to be returned.
     * @return The raw pose Jacobian up to the requested column.
     * @throws std::runtime_error If @p to_link is outside the interval [0, 2].
     */
    MatrixXd raw_pose_jacobian(const VectorXd& q, const int& to_link=2) const;
    /**
     * @brief Computes the raw time derivative of the pose Jacobian of the planar mobile base.
     *
     * @param q Configuration vector [x, y, phi]^T.
     * @param q_dot Configuration-velocity vector [x_dot, y_dot, phi_dot]^T.
     * @param to_link Column index of the partial Jacobian derivative to be returned.
     * @return The raw pose-Jacobian derivative up to the requested column.
     * @throws std::runtime_error If @p to_link is outside the interval [0, 2].
     */
    MatrixXd raw_pose_jacobian_derivative(const VectorXd& q,
                                          const VectorXd& q_dot, const int& to_link = 2) const;
};

}

#endif
