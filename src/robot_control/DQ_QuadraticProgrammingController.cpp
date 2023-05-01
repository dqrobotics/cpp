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

#include <dqrobotics/robot_control/DQ_QuadraticProgrammingController.h>

namespace DQ_robotics
{

DQ_QuadraticProgrammingSolver *DQ_QuadraticProgrammingController::_get_solver_ptr()
{
    return qp_solver_sptr_ ? qp_solver_sptr_.get() : qp_solver_;
}

DQ_QuadraticProgrammingController::DQ_QuadraticProgrammingController(DQ_Kinematics* robot,
                                                                     DQ_QuadraticProgrammingSolver* solver)
    :DQ_KinematicConstrainedController (robot),
      qp_solver_(solver)
{

}

DQ_QuadraticProgrammingController::DQ_QuadraticProgrammingController(const std::shared_ptr<DQ_Kinematics> &robot,
                                                                     const std::shared_ptr<DQ_QuadraticProgrammingSolver> &solver):
    DQ_KinematicConstrainedController(robot),
    qp_solver_sptr_(solver)
{

}

VectorXd DQ_QuadraticProgrammingController::compute_setpoint_control_signal(const VectorXd &q, const VectorXd &task_reference)
{
    return DQ_QuadraticProgrammingController::compute_tracking_control_signal(q,task_reference,VectorXd::Zero(task_reference.size()));
}

VectorXd DQ_QuadraticProgrammingController::compute_tracking_control_signal(const VectorXd &q, const VectorXd &task_reference, const VectorXd &feed_forward)
{
    if(is_set())
    {
        const VectorXd task_variable = get_task_variable(q);

        const MatrixXd J = get_jacobian(q);

        if(task_variable.size() != task_reference.size())
            throw std::runtime_error("Incompatible sizes between task variable and task reference in compute_tracking_control_signal");

        const VectorXd task_error = task_variable - task_reference;

        if(J.rows() != task_error.size())
            throw std::runtime_error("Incompatible sizes between the Jacobian and the task error in compute_tracking_control_signal");
        if(task_error.size() != feed_forward.size())
            throw std::runtime_error("Incompatible sizes between task error and feedforward in compute_tracking_control_signal");

        const MatrixXd& A = inequality_constraint_matrix_;
        const VectorXd& b = inequality_constraint_vector_;
        const MatrixXd& Aeq = equality_constraint_matrix_;
        const VectorXd& beq = equality_constraint_vector_;

        const MatrixXd H = compute_objective_function_symmetric_matrix(J,task_error - (1.0/gain_)*feed_forward);
        const MatrixXd f = compute_objective_function_linear_component(J,task_error - (1.0/gain_)*feed_forward);

        VectorXd u = _get_solver_ptr()->solve_quadratic_program(H,f,A,b,Aeq,beq);

        verify_stability(task_error);

        last_control_signal_ = u;
        last_error_signal_   = task_error;

        return u;
    }
    {
        throw std::runtime_error("Trying to compute the control signal using an unset controller");
    }
}

}

