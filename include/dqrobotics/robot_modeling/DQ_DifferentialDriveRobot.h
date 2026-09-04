/**
(C) Copyright 2019-2022 DQ Robotics Developers

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

#ifndef DQ_ROBOTICS_ROBOT_MODELING_DQ_DIFFERENTIALDRIVEROBOT
#define DQ_ROBOTICS_ROBOT_MODELING_DQ_DIFFERENTIALDRIVEROBOT

#include<dqrobotics/DQ.h>
#include<dqrobotics/robot_modeling/DQ_HolonomicBase.h>

namespace DQ_robotics
{

/**
 * @brief Basic implementation of a differential-drive mobile robot.
 *
 * The robot pose is modeled as a holonomic base with configuration
 * q = [x, y, phi]^T, while the actuation is described by the angular
 * velocities of the right and left wheels. The pose Jacobians in this class
 * already account for the nonholonomic rolling constraint.
 *
 * @see DQ_HolonomicBase
 */
class DQ_DifferentialDriveRobot : public DQ_HolonomicBase
{
protected:
    /** @brief Radius of each wheel, in meters. */
    double wheel_radius_;
    /** @brief Distance between the wheels, in meters. */
    double distance_between_wheels_;
public:
    /**
     * @brief Constructs a differential-drive robot.
     *
     * @param wheel_radius Radius of each wheel, in meters.
     * @param distance_between_wheels Distance between the wheels, in meters.
     */
    DQ_DifferentialDriveRobot(const double& wheel_radius, const double& distance_between_wheels);

    /**
     * @brief Computes the constraint Jacobian relating wheel velocities to configuration velocities.
     *
     * The returned matrix satisfies [x_dot, y_dot, phi_dot]^T = J * [wr, wl]^T,
     * where wr and wl are the right- and left-wheel angular velocities.
     *
     * @param phi Planar orientation of the robot.
     * @return The differential-drive constraint Jacobian.
     */
    MatrixXd constraint_jacobian(const double& phi) const;
    /**
     * @brief Computes the time derivative of the differential-drive constraint Jacobian.
     *
     * @param phi Planar orientation of the robot.
     * @param phi_dot Time derivative of @p phi.
     * @return The time derivative of the differential-drive constraint Jacobian.
     */
    MatrixXd constraint_jacobian_derivative(const double& phi, const double& phi_dot) const;
    /**
     * @brief Computes the constrained pose Jacobian.
     *
     * The returned Jacobian maps wheel angular velocities to the time derivative
     * of the base pose. Because there are two independent wheel velocities, the
     * valid values of @p to_link are 0 and 1.
     *
     * @param q Configuration vector [x, y, phi]^T.
     * @param to_link Column index of the partial Jacobian to be returned.
     * @return The constrained pose Jacobian up to the requested column.
     * @throws std::runtime_error If @p to_link is not 0 or 1.
     */
    MatrixXd pose_jacobian(const VectorXd& q, const int& to_link) const override;
    /**
     * @brief Computes the full constrained pose Jacobian.
     *
     * @param q Configuration vector [x, y, phi]^T.
     * @return The full constrained pose Jacobian.
     */
    MatrixXd pose_jacobian(const VectorXd &q) const override;
    /**
     * @brief Computes the time derivative of the constrained pose Jacobian.
     *
     * @param q Configuration vector [x, y, phi]^T.
     * @param q_dot Configuration-velocity vector [x_dot, y_dot, phi_dot]^T.
     * @param to_link Column index of the partial Jacobian derivative to be returned.
     * @return The constrained pose-Jacobian derivative up to the requested column.
     * @throws std::runtime_error If @p to_link is not 0 or 1.
     */
    MatrixXd pose_jacobian_derivative(const VectorXd& q,
                                      const VectorXd& q_dot,
                                      const int& to_link) const override;
    /**
     * @brief Computes the full time derivative of the constrained pose Jacobian.
     *
     * @param q Configuration vector [x, y, phi]^T.
     * @param q_dot Configuration-velocity vector [x_dot, y_dot, phi_dot]^T.
     * @return The full constrained pose-Jacobian derivative.
     */
    MatrixXd pose_jacobian_derivative (const VectorXd& q,
                                       const VectorXd& q_dot) const override;
};

}

#endif
