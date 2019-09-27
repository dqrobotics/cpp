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

#ifndef DQ_ROBOT_CONTROL_DQ_TASKSPACEPSEUDOINVERSECONTROLLER_H
#define DQ_ROBOT_CONTROL_DQ_TASKSPACEPSEUDOINVERSECONTROLLER_H

#include <memory>

#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_control/DQ_KinematicController.h>

namespace DQ_robotics
{

class DQ_TaskSpacePseudoInverseController: public DQ_KinematicController
{
public:
    //Remove default constructor
    DQ_TaskSpacePseudoInverseController() = delete;

    DQ_TaskSpacePseudoInverseController(DQ_Kinematics* robot);

    VectorXd compute_setpoint_control_signal(const VectorXd& q, const VectorXd& task_reference);
    VectorXd compute_tracking_control_signal(const VectorXd& q, const VectorXd& task_reference, const VectorXd& feed_forward);
};

}

#endif
