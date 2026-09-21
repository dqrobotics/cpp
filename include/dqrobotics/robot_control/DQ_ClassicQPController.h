#pragma once
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



#include<dqrobotics/robot_control/DQ_QuadraticProgrammingController.h>

namespace DQ_robotics
{

/**
 * @brief Implements the classic quadratic-programming kinematic controller based on task-space variables.
 *
 * This controller uses a quadratic objective built from the task Jacobian and the
 * Euclidean task-space error. Its default isotropic damping is initialized to 1e-3.
 *
 * @see DQ_QuadraticProgrammingController, DQ_PseudoinverseController
 */
class DQ_ClassicQPController:public DQ_QuadraticProgrammingController
{
public:
    DQ_ClassicQPController() = delete;

    //Deprecated
    /**
     * @brief Constructs a classic QP controller from legacy raw pointers.
     *
     * @param robot Non-owning pointer to the robot kinematic model.
     * @param solver Non-owning pointer to the quadratic-programming solver.
     */
    DQ_ClassicQPController(DQ_Kinematics* robot, DQ_QuadraticProgrammingSolver* solver);
    /**
     * @brief Constructs a classic QP controller from shared pointers.
     *
     * @param robot Shared pointer to the robot kinematic model.
     * @param solver Shared pointer to the quadratic-programming solver.
     */
    DQ_ClassicQPController(const std::shared_ptr<DQ_Kinematics>& robot,
                           const std::shared_ptr<DQ_QuadraticProgrammingSolver>& solver);

    /**
     * @brief Computes the symmetric matrix H used in the quadratic objective.
     *
     * The returned matrix corresponds to J'J plus isotropic damping. The second
     * argument of the abstract interface is unused by this controller.
     *
     * @param J Task Jacobian associated with the current control objective.
     * @return The symmetric matrix H of the quadratic objective.
     */
    MatrixXd compute_objective_function_symmetric_matrix(const MatrixXd& J, const VectorXd&) override;
    /**
     * @brief Computes the linear vector f used in the quadratic objective.
     *
     * @param J Task Jacobian associated with the current control objective.
     * @param task_error Current task-space error.
     * @return The linear component f of the quadratic objective.
     */
    VectorXd compute_objective_function_linear_component(const MatrixXd& J, const VectorXd& task_error) override;
};

}
