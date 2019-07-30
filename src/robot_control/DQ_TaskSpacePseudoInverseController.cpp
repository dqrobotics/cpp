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
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
*/
#include <dqrobotics/robot_control/DQ_TaskSpacePseudoInverseController.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>

namespace DQ_robotics
{

VectorXd DQ_TaskSpacePseudoInverseController::compute_setpoint_control_signal(const VectorXd& q, const VectorXd& task_reference)
{
    if(is_set())
    {
        const VectorXd task_variable = get_task_variable(q);

        const MatrixXd J = get_jacobian(q);

        const VectorXd task_error = task_reference-task_variable;

        const VectorXd u = pinv(J)*gain_*task_error;

        if((last_error_signal_-task_error).norm() < stability_threshold_)
        {
            is_stable_ = true;
        }

        last_control_signal_ = u;
        last_error_signal_   = task_error;

        return u;
    }

    throw std::runtime_error("Tried computing the control signal of an unset controller.");
}

VectorXd DQ_TaskSpacePseudoInverseController::compute_tracking_control_signal(const VectorXd &q, const VectorXd &task_reference, const VectorXd &feed_forward)
{
    return compute_setpoint_control_signal(q,task_reference);
}

}

