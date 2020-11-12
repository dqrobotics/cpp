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

#include<dqrobotics/robot_modeling/DQ_DifferentialDriveRobot.h>

namespace DQ_robotics
{

DQ_DifferentialDriveRobot::DQ_DifferentialDriveRobot(const double& wheel_radius, const double& distance_between_wheels)
{
    wheel_radius_            = wheel_radius;
    distance_between_wheels_ = distance_between_wheels;
}

MatrixXd DQ_DifferentialDriveRobot::constraint_jacobian(const double &phi) const
{
    const double& r = wheel_radius_;
    const double& l = distance_between_wheels_;
    double c = cos(phi);
    double s = sin(phi);

    MatrixXd J(3,2);
    J << (r/2)*c, (r/2)*c,
            (r/2)*s, (r/2)*s,
            r/l, -r/l;
    return J;
}

MatrixXd DQ_DifferentialDriveRobot::pose_jacobian(const VectorXd &q, const int &to_link) const
{
    if(to_link!=0 && to_link!=1)
        throw std::runtime_error("DQ_DifferentialDriveRobot::pose_jacobian(q,to_link) only accepts to_link in {0,1}.");

    MatrixXd J_holonomic = DQ_HolonomicBase::pose_jacobian(q,2);
    MatrixXd J = J_holonomic*constraint_jacobian(q(2));
    return J.block(0,0,8,to_link+1);
}

MatrixXd DQ_DifferentialDriveRobot::pose_jacobian(const VectorXd &q) const
{
    // The DQ_DifferentialDriveRobot works differently from most other subclasses of DQ_Kinematics
    // The size of the configuration space is three but there is one constraint, so there are only
    // to columns in the pose_jacobian
    return pose_jacobian(q,1);
}

}
