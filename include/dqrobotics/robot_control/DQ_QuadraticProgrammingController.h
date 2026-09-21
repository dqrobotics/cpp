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

#include<dqrobotics/robot_control/DQ_KinematicConstrainedController.h>
#include<dqrobotics/solvers/DQ_QuadraticProgrammingSolver.h>

using namespace Eigen;

namespace DQ_robotics
{
/**
 * @brief Abstract class that defines task-space kinematic controllers based on quadratic programming.
 *
 * Although many kinematic controllers can be written as quadratic programs, this class
 * targets formulations whose objective function depends on task-space quantities such as
 * the robot Jacobian and the task-space error. Subclasses define the symmetric and linear
 * terms of the quadratic objective, while this class assembles the constrained optimization
 * problem and delegates it to a DQ_QuadraticProgrammingSolver.
 *
 * @note This class remains abstract because subclasses must implement the objective-function terms.
 * @see DQ_KinematicConstrainedController, DQ_ClassicQPController, DQ_QuadraticProgrammingSolver
 */
class DQ_QuadraticProgrammingController:public DQ_KinematicConstrainedController
{
protected:
    //Deprecated together with the raw pointer constructors but without the C++14 attribute as it is too noisy.
    /** @brief Legacy non-owning pointer to the quadratic-programming solver kept for backwards compatibility. */
    DQ_QuadraticProgrammingSolver* qp_solver_;
    /** @brief Shared pointer to the quadratic-programming solver used by the smart-pointer constructors. */
    std::shared_ptr<DQ_QuadraticProgrammingSolver> qp_solver_sptr_;

    /**
     * @brief Returns the active quadratic-programming solver pointer.
     *
     * @return Pointer to the solver used to solve the control problem.
     */
    DQ_QuadraticProgrammingSolver* _get_solver_ptr();

    /**
     * @brief Constructs a controller from legacy raw pointers.
     *
     * @param robot Non-owning pointer to the robot kinematic model.
     * @param solver Non-owning pointer to the quadratic-programming solver.
     */
    [[deprecated("Use the smart pointer version instead")]]
    DQ_QuadraticProgrammingController(DQ_Kinematics *robot, DQ_QuadraticProgrammingSolver *solver);
    /**
     * @brief Constructs a controller from shared pointers.
     *
     * @param robot Shared pointer to the robot kinematic model.
     * @param solver Shared pointer to the quadratic-programming solver.
     */
    DQ_QuadraticProgrammingController(const std::shared_ptr<DQ_Kinematics>& robot,
                                      const std::shared_ptr<DQ_QuadraticProgrammingSolver>& solver);
public:
    //Remove default constructor
    DQ_QuadraticProgrammingController()=delete;

    /**
     * @brief Computes the symmetric matrix H of the quadratic objective.
     *
     * Pure virtual interface contract implemented by concrete quadratic-programming controllers.
     *
     * @param J Task Jacobian associated with the current control objective.
     * @param task_error Task-space error.
     * @return The symmetric matrix H in the quadratic objective.
     */
    virtual MatrixXd compute_objective_function_symmetric_matrix(const MatrixXd& J, const VectorXd& task_error)=0;
    /**
     * @brief Computes the linear vector f of the quadratic objective.
     *
     * Pure virtual interface contract implemented by concrete quadratic-programming controllers.
     *
     * @param J Task Jacobian associated with the current control objective.
     * @param task_error Task-space error.
     * @return The linear component f in the quadratic objective.
     */
    virtual VectorXd compute_objective_function_linear_component(const MatrixXd& J, const VectorXd& task_error)=0;

    /**
     * @brief Computes the control signal for a setpoint task.
     *
     * @param q Vector containing the current joint configurations of the robot.
     * @param task_reference Vector containing the desired value for the chosen control task.
     * @return The reference joint velocities obtained from the quadratic program.
     * @throws std::runtime_error If the controller is unset or if incompatible task, Jacobian, or feedforward sizes are detected.
     */
    virtual VectorXd compute_setpoint_control_signal(const VectorXd&q, const VectorXd& task_reference) override;
    /**
     * @brief Computes the reference joint velocities for a tracking task with feedforward.
     *
     * The optimization problem uses the stored equality and inequality constraints together
     * with the objective-function terms provided by the concrete subclass.
     *
     * @param q Vector containing the current joint configurations of the robot.
     * @param task_reference Vector containing the desired value for the chosen control task.
     * @param feed_forward Time derivative of the task reference expressed in task space.
     * @return The reference joint velocities obtained from the quadratic program.
     * @throws std::runtime_error If the controller is unset or if incompatible task, Jacobian, or feedforward sizes are detected.
     */
    virtual VectorXd compute_tracking_control_signal(const VectorXd&q, const VectorXd& task_reference, const VectorXd& feed_forward) override;

};
}
