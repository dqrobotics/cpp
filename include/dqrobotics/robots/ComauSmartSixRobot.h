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

#ifndef DQ_ROBOTS_COMAUSMARTSIXROBOT_H
#define DQ_ROBOTS_COMAUSMARTSIXROBOT_H

#include<dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>

namespace DQ_robotics
{

/**
 * @brief Provides the kinematic model of the COMAU SmartSiX robot manipulator.
 *
 * This class exposes a ready-to-use model of the COMAU SmartSiX robot based on
 * the Denavit-Hartenberg parameters used in the MATLAB version of DQ Robotics.
 */
class ComauSmartSixRobot
{
public:
    /**
     * @brief Returns the kinematic model of the COMAU SmartSiX robot manipulator.
     * @return A DQ_SerialManipulatorDH instance representing the robot.
     * @note The MATLAB reference model is described using the modified
     * Denavit-Hartenberg convention.
     */
    static DQ_SerialManipulatorDH kinematics();
};
}

#endif
