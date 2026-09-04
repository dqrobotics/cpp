/**
(C) Copyright 2022 DQ Robotics Developers
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
- Juan Jose Quiroz Omana (juanjqo@g.ecc.u-tokyo.ac.jp)
*/

#pragma once
#include<dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>

namespace DQ_robotics
{

/**
 * @brief Provides the kinematic model of the Franka Emika Panda robot manipulator.
 *
 * This class exposes a ready-to-use model of the Franka Emika Panda arm using
 * the geometric data encoded in the library implementation.
 */
class FrankaEmikaPandaRobot
{
public:
    /**
     * @brief Returns the kinematic model of the Franka Emika Panda robot,
     * described using the modified Denavit-Hartenberg convention.
     * @return A DQ_SerialManipulatorMDH instance representing the robot.
     * @note The implementation also sets the manufacturer base and flange
     * offsets, as well as the joint position and velocity limits.
     */
    static DQ_SerialManipulatorMDH kinematics();
    //static DQ_SerialManipulatorMDH dynamics(); To be implemented
};

}

