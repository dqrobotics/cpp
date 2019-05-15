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
#include<dqrobotics/robots/ComauSmartSixRobot.h>
#include<dqrobotics/utils/DQ_Constants.h>

namespace DQ_robotics
{

DQ_SerialManipulator ComauSmartSixRobot::kinematics()
{
    const double pi2 = pi/2.0;

    Matrix<double,5,7> comau_dh(5,7);
    comau_dh << 0,   -pi2,   pi2,    0,       0,    0,     pi,
            -0.45, 0,     0,     -0.64707, 0,   -0.095, 0,
            0,    0.150, 0.590,  0.13,    0,    0,     0,
            pi,   pi2,   pi,    -pi2,    -pi2,  pi2,   pi,
            0,    0,     0,      0,       0,    0,     1;

    DQ_SerialManipulator comau(comau_dh,"standard");

    return comau;
};

}
