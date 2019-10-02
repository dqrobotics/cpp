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

#include <dqrobotics/robot_control/DQ_ClassicQPController.h>

namespace DQ_robotics
{

    DQ_ClassicQPController::DQ_ClassicQPController(DQ_Kinematics* robot, DQ_QuadraticProgrammingSolver* solver):DQ_QuadraticProgrammingController (robot,solver)
    {
        //Nothing to do
    }

    MatrixXd DQ_ClassicQPController::compute_objective_function_symmetric_matrix(const MatrixXd &J, const VectorXd &task_error)
    {
        return (J.transpose()*J + damping_*MatrixXd::Identity(robot_->get_dim_configuration_space(),robot_->get_dim_configuration_space()));
    }

    VectorXd DQ_ClassicQPController::compute_objective_function_linear_component(const MatrixXd &J, const VectorXd &task_error)
    {
        return gain_*J.transpose()*task_error;
    }
}

