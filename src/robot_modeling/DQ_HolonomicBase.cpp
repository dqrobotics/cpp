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

#include<dqrobotics/robot_modeling/DQ_HolonomicBase.h>

namespace DQ_robotics
{

DQ_HolonomicBase::DQ_HolonomicBase()
{
    dim_configuration_space_ = 3;
}

DQ DQ_HolonomicBase::raw_fkm(const VectorXd& q) const
{
    const double& x   = q(0);
    const double& y   = q(1);
    const double& phi = q(2);

    double c = cos(phi/2.0);
    double s = sin(phi/2.0);

    DQ real_part = c + k_*s;
    DQ dual_part = 0.5*i_*(x*c + y*s) + 0.5*j_*(-x*s + y*c);

    return real_part + E_*dual_part;
}

DQ DQ_HolonomicBase::fkm(const VectorXd& q) const
{
    return raw_fkm(q)*frame_displacement_;
}

DQ DQ_HolonomicBase::fkm(const VectorXd &q, const int& to_ith_link) const
{
    if(to_ith_link != 2)
    {
        throw std::runtime_error(std::string("DQ_HolonomicBase(q,to_ith_link) only accepts to_ith_link=2"));
    }
    return fkm(q);
}

MatrixXd DQ_HolonomicBase::raw_pose_jacobian(const VectorXd& q, const int& to_link) const
{
    if(to_link >= 3 || to_link < 0)
    {
        throw std::runtime_error(std::string("Tried to access link index ") + std::to_string(to_link) + std::string(" which is unnavailable."));
    }

    const double& x   = q(0);
    const double& y   = q(1);
    const double& phi = q(2);

    const double c = cos(phi/2.0);
    const double s = sin(phi/2.0);

    const double j71 = -0.5*s;
    const double j62 = -j71;
    const double j13 = -j62;

    const double j72 = 0.5*c;
    const double j61 = j72;
    const double j43 = j61;

    const double j63 = 0.25*(-x*s + y*c);

    const double j73 = 0.25*(-x*c - y*s);

    MatrixXd J(8,3);
    J <<    0.0, 0.0, j13,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, j43,
            0.0, 0.0, 0.0,
            j61, j62, j63,
            j71, j72, j73,
            0.0, 0.0, 0.0;
    return J.block(0,0,8,to_link+1);
}

MatrixXd DQ_HolonomicBase::pose_jacobian(const VectorXd &q, const int &to_link) const
{
    return haminus8(frame_displacement_)*raw_pose_jacobian(q,to_link);
}

MatrixXd DQ_HolonomicBase::pose_jacobian(const VectorXd &q) const
{
    return pose_jacobian(q,get_dim_configuration_space()-1);
}

/**
 * @brief returns the Jacobian derivative (J_dot) that satisfies
          vec8(pose_dot_dot) = J_dot * q_dot + J*q_dot_dot, where pose = fkm(), 'pose_dot' is the time
          derivative of the pose and 'q_dot' represents the robot configuration velocities.
          This method does not take into account the base displacement.
 * @param q The VectorXd representing the robot configurations.
 * @param q_dot The VectorXd representing the robot configuration velocities.
 * @param to_link The ith link which we want to compute the Jacobian derivative.
 * @return a MatrixXd representing the desired Jacobian derivative.
 */
MatrixXd DQ_HolonomicBase::raw_pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot, const int &to_link) const
{
    if(to_link >= 3 || to_link < 0)
    {
        throw std::runtime_error(std::string("Tried to access link index ") + std::to_string(to_link) + std::string(" which is unnavailable."));
    }

    const double& x   = q(0);
    const double& y   = q(1);
    const double& phi = q(2);

    const double& x_dot   = q_dot(0);
    const double& y_dot   = q_dot(1);
    const double& phi_dot = q_dot(2);

    const double c = cos(phi/2.0);
    const double s = sin(phi/2.0);

    const double j71 = -0.25*c*phi_dot;
    const double j62 = -j71;
    const double j13 = -j62;

    const double j72 = -0.25*s*phi_dot;
    const double j61 = j72;
    const double j43 = j61;

    const double j63 = 0.25 * (-x_dot*s - 0.5*x*c*phi_dot + y_dot*c - 0.5*y*s*phi_dot);
    const double j73 = 0.25 * (-x_dot*c + 0.5*x*s*phi_dot - y_dot*s - 0.5*y*c*phi_dot);

    MatrixXd J_dot(8,3);
    J_dot <<    0.0, 0.0, j13,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, j43,
            0.0, 0.0, 0.0,
            j61, j62, j63,
            j71, j72, j73,
            0.0, 0.0, 0.0;
    return J_dot.block(0,0,8,to_link+1);
}

/**
 * @brief returns the Jacobian derivative 'J_dot' that satisfies
          vec8(pose_dot_dot) = J_dot * q_dot + J*q_dot_dot, where pose = fkm(), 'pose_dot' is the time
          derivative of the pose and 'q_dot' represents the robot configuration velocities.
 * @param q The VectorXd representing the robot configurations.
 * @param q_dot The VectorXd representing the robot configuration velocities.
 * @param to_ith_link The 'to_ith_link' link which we want to compute the Jacobian derivative.
 * @return a MatrixXd representing the desired Jacobian derivative.
 */
MatrixXd DQ_HolonomicBase::pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot, const int &to_link) const
{
    return haminus8(frame_displacement_)*raw_pose_jacobian_derivative(q, q_dot, to_link);
}

/**
 * @brief returns the Jacobian derivative 'J_dot' that satisfies
          vec8(pose_dot_dot) = J_dot * q_dot + J*q_dot_dot, where pose = fkm(), 'pose_dot' is the time
          derivative of the pose and 'q_dot' represents the robot configuration velocities.
 * @param q The VectorXd representing the robot configurations.
 * @param q_dot The VectorXd representing the robot configuration velocities.
 * @return a MatrixXd representing the desired Jacobian derivative.
 */
MatrixXd DQ_HolonomicBase::pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot) const
{
    return pose_jacobian_derivative(q, q_dot, get_dim_configuration_space()-1);
}

}
