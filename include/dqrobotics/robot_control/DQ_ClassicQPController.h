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
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
*/



#include<dqrobotics/robot_control/DQ_QuadraticProgrammingController.h>

namespace DQ_robotics
{

class DQ_ClassicQPController:public DQ_QuadraticProgrammingController
{
public:
    DQ_ClassicQPController() = delete;

    //Deprecated
    DQ_ClassicQPController(DQ_Kinematics* robot, DQ_QuadraticProgrammingSolver* solver);
    DQ_ClassicQPController(const std::shared_ptr<DQ_Kinematics>& robot,
                           const std::shared_ptr<DQ_QuadraticProgrammingSolver>& solver);

    MatrixXd compute_objective_function_symmetric_matrix(const MatrixXd& J, const VectorXd&) override;
    VectorXd compute_objective_function_linear_component(const MatrixXd& J, const VectorXd& task_error) override;
};

}
