/**
(C) Copyright 2019-2022 DQ Robotics Developers

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

/**
 * @brief returns the time derivative of the constraint Jacobian
 * @param phi The orientation of the robot on the plane.
 * @param phi_dot The time derivative of phi.
 * @return a MatrixXd representing the desired Jacobian derivative
 */
MatrixXd DQ_DifferentialDriveRobot::constraint_jacobian_derivative(const double &phi, const double &phi_dot) const
{
    const double& r = wheel_radius_;
    double c = cos(phi);
    double s = sin(phi);

    MatrixXd J_dot(3,2);
    J_dot << -(r/2)*s, -(r/2)*s,
            (r/2)*c, (r/2)*c,
               0, 0;
    return J_dot*phi_dot;
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


/**
 * @brief returns the pose Jacobian derivative
 * @param configurations The VectorXd representing the robot configuration.
 * @param velocity_configurations The VectorXd representing the robot configuration velocity.
 * @param to_link The ith link which we want to compute the Jacobian derivative.
 * @return a MatrixXd representing the desired Jacobian
 */
MatrixXd DQ_DifferentialDriveRobot::pose_jacobian_derivative(const VectorXd &configurations, const VectorXd &velocity_configurations, const int &to_link) const
{
    if(to_link!=0 && to_link!=1)
        throw std::runtime_error("DQ_DifferentialDriveRobot::pose_jacobian_derivative(q,q_dot, to_link) only accepts to_link in {0,1}.");

    /// Aliases
    const VectorXd& q = configurations;
    const VectorXd& q_dot = velocity_configurations;

    /// Requirements
    MatrixXd J_holonomic = DQ_HolonomicBase::pose_jacobian(q,2);
    MatrixXd J_holonomic_dot = DQ_HolonomicBase::pose_jacobian_derivative(q,q_dot,2);
    MatrixXd J_c = constraint_jacobian(q(2));
    MatrixXd J_c_dot = constraint_jacobian_derivative(q(2), q_dot(2));
    MatrixXd J_dot = J_holonomic_dot*J_c + J_holonomic*J_c_dot;
    return J_dot.block(0,0,8,to_link+1);
}

/**
 * @brief returns the pose Jacobian derivative
 * @param configurations The VectorXd representing the robot configuration.
 * @param velocity_configurations The VectorXd representing the robot configuration velocity.
 * @return a MatrixXd representing the desired Jacobian
 */
MatrixXd DQ_DifferentialDriveRobot::pose_jacobian_derivative(const VectorXd &configurations, const VectorXd &velocity_configurations) const
{
    // The DQ_DifferentialDriveRobot works differently from most other subclasses of DQ_Kinematics
    // The size of the configuration space is three but there is one constraint, so there are only
    // to columns in the pose_jacobian

    /// Aliases
    const VectorXd& q = configurations;
    const VectorXd& q_dot = velocity_configurations;
    return pose_jacobian_derivative(q, q_dot, 1);
}

}
