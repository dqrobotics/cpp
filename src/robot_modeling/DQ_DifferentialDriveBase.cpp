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

#include<dqrobotics/robot_modeling/DQ_DifferentialDriveBase.h>

namespace DQ_robotics
{

DQ_DifferentialDriveRobot::DQ_DifferentialDriveRobot(const double& wheel_radius, const double& distance_between_wheels)
{
    wheel_radius_            = wheel_radius;
    distance_between_wheels_ = distance_between_wheels;
}

MatrixXd DQ_DifferentialDriveRobot::constraint_jacobian(const double &phi)
{
    double& r = wheel_radius_;
    double& l = distance_between_wheels_;
    double c = cos(phi);
    double s = sin(phi);

    MatrixXd J(3,2);
    J << (r/2)*c, (r/2)*c,
            (r/2)*s, (r/2)*s,
            r/l, -r/l;
    return J;
}

MatrixXd DQ_DifferentialDriveRobot::pose_jacobian(const VectorXd &q, const int &to_link)
{
    MatrixXd J_holonomic = DQ_MobileBase::pose_jacobian(q,3);
    MatrixXd J = J_holonomic*constraint_jacobian(q(2));
    return J.block(0,0,3,to_link);
}

}
