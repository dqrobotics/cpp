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

#include <dqrobotics/robot_control/DQ_KinematicConstrainedController.h>

namespace DQ_robotics
{

DQ_KinematicConstrainedController::DQ_KinematicConstrainedController(DQ_Kinematics* robot):DQ_KinematicController(robot)
{
    //Nothing to do
}

void DQ_KinematicConstrainedController::set_equality_constraint(const MatrixXd &B, const VectorXd &b)
{
    equality_constraint_matrix_ = B;
    equality_constraint_vector_ = b;
}

void DQ_KinematicConstrainedController::set_inequality_constraint(const MatrixXd &B, const VectorXd &b)
{
    inequality_constraint_matrix_ = B;
    inequality_constraint_vector_ = b;
}

}
