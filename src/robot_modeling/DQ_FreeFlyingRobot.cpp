/**
(C) Copyright 2023 DQ Robotics Developers

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

    1. Bruno Vilhena Adorno (adorno@ieee.org)
       Responsible for the original implementation in file DQ_FreeFlyingRobot.m
       https://github.com/dqrobotics/matlab/pull/66/commits

    2. Juan Jose Quiroz Omana   (juanjqo@g.ecc.u-tokyo.ac.jp)
       Created this file.
*/

#include <dqrobotics/robot_modeling/DQ_FreeFlyingRobot.h>

namespace DQ_robotics
{

/**
 * @brief Constructor of the DQ_FreeFlyingRobot() class.
 */
DQ_FreeFlyingRobot::DQ_FreeFlyingRobot()
{
    dim_configuration_space_ = 8;
}


/**
 * @brief Internal method to handle invalid signatures. This method
 *        throws an exception with the following message:
 *
 *        "This signature is not supported in this class. "
 *        msg + ", where pose is a unit dual quaternion, which "
 *        "represents the free-flying robot configuration. "
 *
 * @param msg Message to provide the valid signature. Ex:
 *
 *         auto msg = std::string("Use fkm(pose)");
 *         _invalid_signature(msg);
 */
void DQ_FreeFlyingRobot::_invalid_signature(const std::string &msg) const
{
    throw std::runtime_error(std::string("This signature is not supported in this class. ") +
                             msg + std::string(", where pose is a unit dual quaternion, which ")+
                             std::string("represents the free-flying robot configuration. "));
}


DQ DQ_FreeFlyingRobot::fkm(const VectorXd&) const
{
    _invalid_signature(std::string("Use fkm(pose)"));
    return DQ(1);
}


DQ DQ_FreeFlyingRobot::fkm(const VectorXd&, const int&) const
{
    _invalid_signature(std::string("Use fkm(pose)"));
    return DQ(1);
}

MatrixXd DQ_FreeFlyingRobot::pose_jacobian(const VectorXd&, const int&) const
{
    _invalid_signature(std::string("Use pose_jacobian(pose)"));
    return Eigen::MatrixXd::Zero(2,2);
}

MatrixXd DQ_FreeFlyingRobot::pose_jacobian_derivative(const VectorXd&, const VectorXd&, const int&) const
{
    _invalid_signature(std::string("Use pose_jacobian_derivative(pose_derivative). "
                                   "pose_derivative denotes the time derivative of pose"));
    return Eigen::MatrixXd::Zero(2,2);
}


/**
 * @brief This method returns the free-flying robot pose.
 * @param pose The unit dual quaternion that represents
 *             the free-flying robot configuration.
 * @return pose The free-flying robot pose.
 */
DQ DQ_FreeFlyingRobot::fkm(const DQ& pose) const
{
    return pose;
}

/**
 * @brief This method returns the pose Jacobian J that satisfies
 *
 *        x_dot = J*twist,
 *
 *        where x_dot is the time derivative of the unit dual quaternion
 *        that represents the free-flying robot pose, and twist = vec8(csi),
 *        where csi satisfies the dual quaternion propagation equation
 *        xdot = (1/2)*csi*x.
 *
 * @param pose The unit dual quaternion that represents
 *             the free-flying robot configuration.
 * @return The desired pose Jacobian.
 */
MatrixXd DQ_FreeFlyingRobot::pose_jacobian(const DQ &pose) const
{
    return haminus8(0.5*pose);
}


/**
 * @brief This method returns the pose Jacobian derivative of J, where
 *        J is the pose Jacobian that satisfies
 *
 *        x_dot = J*twist,
 *
 *        where x_dot is the time derivative of the unit dual quaternion
 *        that represents the free-flying robot pose, and twist = vec8(csi),
 *        where csi satisfies the dual quaternion propagation equation
 *        xdot = (1/2)*csi*x.
 *
 * @param pose The unit dual quaternion that represents
 *             the free-flying robot configuration.
 * @param pose_derivative The time derivative of the robot configuration.
 * @return The desired pose Jacobian derivative.
 */
MatrixXd DQ_FreeFlyingRobot::pose_jacobian_derivative(const DQ &pose_derivative) const
{
    return haminus8(0.5*pose_derivative);
}

}
