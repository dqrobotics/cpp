/**
(C) Copyright 2011-2026 DQ Robotics Developers

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

1. Murilo M. Marinho (murilomarinho@ieee.org)
    - Responsible for the original implementation.

2. Juan Jose Quiroz Omana (juanjqogm@gmail.com)
    - Added support for smart pointers 
*/

#ifndef DQ_COOPERATIVEDUALTASKSPACE_H
#define DQ_COOPERATIVEDUALTASKSPACE_H

#include<dqrobotics/DQ.h>
#include<dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <memory>

namespace DQ_robotics
{

/**
 * @brief Implements the cooperative dual task-space formulation for two robots.
 *
 * The cooperative variables are the absolute pose and the relative pose of the
 * two end effectors, together with their corresponding Jacobians. The combined
 * configuration vector is assumed to be theta = [q1; q2], where q1 and q2 are
 * the configuration vectors of the first and second robots.
 *
 * @see DQ_Kinematics
 */
class DQ_CooperativeDualTaskSpace
{
private:
    //Deprecated together with the raw pointer constructor but without the C++14 attribute as it is too noisy.
    /** @brief Legacy non-owning pointer to the first robot model kept for backwards compatibility. */
    DQ_Kinematics* robot1_;
    /** @brief Legacy non-owning pointer to the second robot model kept for backwards compatibility. */
    DQ_Kinematics* robot2_;

    /** @brief Shared pointer to the first robot model.*/
    std::shared_ptr<DQ_Kinematics> robot1_sptr_;

    /** @brief Shared pointer to the second robot model.*/
    std::shared_ptr<DQ_Kinematics> robot2_sptr_;

    /**
     * @brief Returns the active pointer to the first robot model.
     *
     * @return Pointer to the first robot model.
     */
    DQ_Kinematics* _get_robot1_ptr() const;
    /**
     * @brief Returns the active pointer to the second robot model.
     *
     * @return Pointer to the second robot model.
     */
    DQ_Kinematics* _get_robot2_ptr() const;

public:
    //Remove default constructor
    /** @brief Deleted default constructor. */
    DQ_CooperativeDualTaskSpace()=delete;

    /**
     * @brief Constructs a cooperative dual task-space system from two robot models.
     *
     * The object does not take ownership of the provided pointers.
     *
     * @param robot1 Pointer to the first robot model.
     * @param robot2 Pointer to the second robot model.
     */
    [[deprecated("Use the smart pointer version instead")]]
    DQ_CooperativeDualTaskSpace(DQ_Kinematics* robot1, DQ_Kinematics* robot2);

    /**
     * @brief Constructs a cooperative dual task-space system from two robot models.
     *
     * The object shares the ownership of the provided pointers.
     *
     * @param robot1 Shared pointer to the first robot model.
     * @param robot2 Shared pointer to the second robot model.
     */
    DQ_CooperativeDualTaskSpace(const std::shared_ptr<DQ_Kinematics>& robot1,
                                const std::shared_ptr<DQ_Kinematics>& robot2);

    /**
     * @brief Returns the pose of the first end effector.
     *
     * @param theta Combined configuration vector [q1; q2].
     * @return Pose of the first end effector.
     */
    DQ pose1(const VectorXd& theta);
    /**
     * @brief Returns the pose of the second end effector.
     *
     * @param theta Combined configuration vector [q1; q2].
     * @return Pose of the second end effector.
     */
    DQ pose2(const VectorXd& theta);

    /**
     * @brief Returns the pose Jacobian of the first robot.
     *
     * @param theta Combined configuration vector [q1; q2].
     * @return Pose Jacobian of the first robot end effector.
     */
    MatrixXd pose_jacobian1(const VectorXd& theta);
    /**
     * @brief Returns the pose Jacobian of the second robot.
     *
     * @param theta Combined configuration vector [q1; q2].
     * @return Pose Jacobian of the second robot end effector.
     */
    MatrixXd pose_jacobian2(const VectorXd& theta);

    /**
     * @brief Computes the relative pose between the two end effectors.
     *
     * The returned dual quaternion maps the second end-effector frame to the first one.
     *
     * @param theta Combined configuration vector [q1; q2].
     * @return Relative pose between the two end effectors.
     */
    DQ relative_pose(const VectorXd& theta);
    /**
     * @brief Computes the absolute pose of the cooperative system.
     *
     * The absolute pose corresponds to a frame located midway between the two end effectors.
     *
     * @param theta Combined configuration vector [q1; q2].
     * @return Absolute pose of the cooperative system.
     */
    DQ absolute_pose(const VectorXd& theta);

    /**
     * @brief Computes the Jacobian of the relative pose.
     *
     * @param theta Combined configuration vector [q1; q2].
     * @return Relative-pose Jacobian.
     */
    MatrixXd relative_pose_jacobian(const VectorXd& theta);
    /**
     * @brief Computes the Jacobian of the absolute pose.
     *
     * @param theta Combined configuration vector [q1; q2].
     * @return Absolute-pose Jacobian.
     */
    MatrixXd absolute_pose_jacobian(const VectorXd& theta);

};


}

#endif
