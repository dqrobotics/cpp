#pragma once
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
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_control/DQ_KinematicController.h>

namespace DQ_robotics
{

/**
 * @brief Implements a kinematic control law based on the Jacobian pseudoinverse and an Euclidean task-space error.
 *
 * This controller computes reference joint velocities from the task-space error using
 * the Moore-Penrose pseudoinverse when the damping is zero, or a damped least-squares
 * inverse when isotropic damping is enabled.
 *
 * @see DQ_KinematicController, DQ_NumericalFilteredPseudoinverseController
 */
class DQ_PseudoinverseController: public DQ_KinematicController
{
public:
    DQ_PseudoinverseController() = delete;

    /**
     * @brief Constructs a controller from a legacy raw robot pointer.
     *
     * @param robot Non-owning pointer to the robot kinematic model.
     */
    [[deprecated("Use the smart pointer version instead")]]
    DQ_PseudoinverseController(DQ_Kinematics* robot);
    /**
     * @brief Constructs a controller from a shared robot pointer.
     *
     * @param robot Shared pointer to the robot kinematic model.
     */
    DQ_PseudoinverseController(const std::shared_ptr<DQ_Kinematics>& robot);

    /**
     * @brief Computes the control signal that regulates the closed-loop system to a setpoint.
     *
     * @param q Vector containing the current joint configurations of the robot.
     * @param task_reference Vector containing the desired value for the chosen control task.
     * @return The reference joint velocities.
     * @throws std::runtime_error If the controller was not configured with a valid control objective.
     */
    VectorXd compute_setpoint_control_signal(const VectorXd& q, const VectorXd& task_reference) override;
    /**
     * @brief Computes the reference joint velocities for a time-varying task-space reference.
     *
     * @param q Vector containing the current joint configurations of the robot.
     * @param task_reference Vector containing the desired value for the chosen control task.
     * @param feed_forward Time derivative of the task reference expressed in task space.
     * @return The reference joint velocities.
     * @throws std::runtime_error If the controller was not configured with a valid control objective.
     */
    VectorXd compute_tracking_control_signal(const VectorXd& q, const VectorXd& task_reference, const VectorXd& feed_forward) override;
};

}
