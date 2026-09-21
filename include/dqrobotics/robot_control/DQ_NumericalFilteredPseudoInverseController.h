#pragma once
//This is an implementation of the controller described in the following work:
//S. Chiaverini,
//"Singularity-robust task-priority redundancy resolution for real-time kinematic control of robot manipulators,"
//in IEEE Transactions on Robotics and Automation,
//vol. 13, no. 3, pp. 398-410, June 1997,
//doi: 10.1109/70.585902.
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
- Murilo M. Marinho (murilomarinho@ieee.org)
*/
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_control/DQ_PseudoinverseController.h>

namespace DQ_robotics
{

/**
 * @brief Implements a singularity-robust pseudoinverse controller with numerical filtered damping.
 *
 * This controller extends DQ_PseudoinverseController with the numerical filtering strategy
 * described by Chiaverini for singularity-robust kinematic control. The Jacobian singular-value
 * decomposition is used to build additional damping only along singular directions whose singular
 * values fall inside the configured singular region. When the singular region size or the maximum
 * numerical filtered damping is zero, when the Jacobian is full rank, or when the filtered damping
 * evaluates to zero, the controller reduces to DQ_PseudoinverseController.
 *
 * @see DQ_PseudoinverseController, DQ_KinematicController
 */
class DQ_NumericalFilteredPseudoinverseController: public DQ_PseudoinverseController
{
protected:
    /** @brief Size of the singular region used to activate numerical filtered damping. */
    double epsilon_; //Size of the singular region, described on the text after Eq. (15)
    /** @brief Maximum numerical filtered damping applied inside the singular region. */
    double lambda_max_; //Maximum value for the numerical filtered damping, described on the text after Eq. (15)
    //double damping_; //(Member variable of DQ_KinematicController: Isotropic damping described on the text above Eq. (20)

    //log
    /** @brief Last filtered damping matrix computed from the Jacobian singular vectors. */
    MatrixXd last_filtered_damping_;
    /** @brief Rank of the last Jacobian processed by the controller. */
    double last_jacobian_rank_;
    /** @brief Singular value decomposition of the last Jacobian, stored as (U, S, V). */
    std::tuple<MatrixXd,MatrixXd,MatrixXd> last_jacobian_svd_;
public:
    DQ_NumericalFilteredPseudoinverseController() = delete;
    /**
     * @brief Constructs a controller from a legacy raw robot pointer.
     *
     * @param robot Non-owning pointer to the robot kinematic model.
     */
    [[deprecated("Use the smart pointer version instead")]]
    DQ_NumericalFilteredPseudoinverseController(DQ_Kinematics* robot);
    /**
     * @brief Constructs a controller from a shared robot pointer.
     *
     * @param robot Shared pointer to the robot kinematic model.
     */
    DQ_NumericalFilteredPseudoinverseController(const std::shared_ptr<DQ_Kinematics>& robot);

    /**
     * @brief Computes the control signal for a setpoint task using numerical filtered damping.
     *
     * @param q Vector containing the current joint configurations of the robot.
     * @param task_reference Vector containing the desired value for the chosen control task.
     * @return The reference joint velocities.
     * @throws std::runtime_error If the controller was not configured with a valid control objective.
     */
    VectorXd compute_setpoint_control_signal(const VectorXd& q, const VectorXd& task_reference) override;
    /**
     * @brief Computes the reference joint velocities for a tracking task using numerical filtered damping.
     *
     * @param q Vector containing the current joint configurations of the robot.
     * @param task_reference Vector containing the desired value for the chosen control task.
     * @param feed_forward Time derivative of the task reference expressed in task space.
     * @return The reference joint velocities.
     * @throws std::runtime_error If the controller was not configured with a valid control objective.
     */
    VectorXd compute_tracking_control_signal(const VectorXd& q, const VectorXd& task_reference, const VectorXd& feed_forward) override;

    /**
     * @brief Sets the maximum numerical filtered damping.
     *
     * @param numerical_filtered_damping Maximum damping value applied inside the singular region.
     */
    void set_maximum_numerical_filtered_damping(const double& numerical_filtered_damping);
    /**
     * @brief Sets the size of the singular region.
     *
     * @param singular_region_size Singular-value threshold that defines the numerical filtering region.
     * @throws std::range_error If @p singular_region_size is negative.
     */
    void set_singular_region_size(const double& singular_region_size);

    /**
     * @brief Returns the maximum numerical filtered damping.
     *
     * @return The configured maximum numerical filtered damping.
     */
    double get_maximum_numerical_filtered_damping() const;
    /**
     * @brief Returns the size of the singular region.
     *
     * @return The configured singular region size.
     */
    double get_singular_region_size() const;
    /**
     * @brief Returns the filtered damping matrix computed in the last control step.
     *
     * @return The last filtered damping matrix.
     */
    MatrixXd get_last_filtered_damping() const;
    /**
     * @brief Returns the rank of the last processed Jacobian.
     *
     * @return The last Jacobian rank.
     * @note The returned value is initialized with -1 before any control signal is computed.
     */
    int get_last_jacobian_rank() const;
    /**
     * @brief Returns the singular value decomposition of the last processed Jacobian.
     *
     * @return A tuple containing the matrices (U, S, V) from the last Jacobian SVD.
     */
    std::tuple<MatrixXd,MatrixXd,MatrixXd> get_last_jacobian_svd() const;
};

}
