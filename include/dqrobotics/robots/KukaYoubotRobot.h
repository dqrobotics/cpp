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

#ifndef DQ_ROBOTS_KUKAYOUBOTROBOT_H
#define DQ_ROBOTS_KUKAYOUBOTROBOT_H

#include<dqrobotics/robot_modeling/DQ_SerialWholeBody.h>

namespace DQ_robotics
{

/**
 * @brief Provides the whole-body kinematic model of the KUKA youBot mobile manipulator.
 *
 * The model is composed of a holonomic mobile base serially coupled to the
 * robot's 5-DOF arm, following the MATLAB version of DQ Robotics.
 */
class KukaYoubotRobot
{
public:
    /**
     * @brief Returns the whole-body kinematic model of the KUKA youBot mobile manipulator.
     * @return A DQ_SerialWholeBody instance representing the holonomic base and
     * the serial arm.
     * @note The arm parameters are described using the standard
     * Denavit-Hartenberg convention, and the geometric dimensions follow KUKA's
     * technical documentation.
     */
    static DQ_SerialWholeBody kinematics();
};

}

#endif
