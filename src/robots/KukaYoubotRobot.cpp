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

#ifndef DQ_ROBOTICS_KUKAYOUBOT_DH_H
#define DQ_ROBOTICS_KUKAYOUBOT_DH_H

#include<dqrobotics/robots/KukaYoubotRobot.h>
#include<dqrobotics/utils/DQ_Constants.h>
#include<dqrobotics/robot_modeling/DQ_HolonomicBase.h>
#include<dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include<dqrobotics/robot_modeling/DQ_WholeBody.h>

namespace DQ_robotics
{

DQ_WholeBody KukaYoubotRobot::kinematics()
{
    const double pi2 = pi/2.0;
    MatrixXd arm_DH_matrix(4,5);
    arm_DH_matrix <<    0,    pi2,       0,      pi2,        0,
                    0.147,      0,       0,        0,    0.218,
                        0,  0.155,   0.135,        0,        0,
                      pi2,      0,       0,      pi2,        0;

    DQ_SerialManipulator arm(arm_DH_matrix,"standard");
    DQ_HolonomicBase     base;

    DQ x_bm= 1 + E_*0.5*(0.22575*i_ + 0.1441*k_);

    base.set_frame_displacement(x_bm);

    DQ_WholeBody robot(&base);
    robot.add(&arm);

    return robot;
}

}

#endif
