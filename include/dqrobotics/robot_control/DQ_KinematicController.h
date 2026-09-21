#pragma once
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
- Murilo M. Marinho (murilomarinho@ieee.org)
*/

#include <memory>

#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>

namespace DQ_robotics
{

/**
 * @brief Enumerates the task-space objectives supported by DQ_KinematicController.
 *
 * @see DQ_KinematicController
 */
enum ControlObjective
{
    /** @brief No control objective has been selected yet. */
    None,
    /** @brief Control the squared distance between the end-effector translation and the origin. */
    Distance,
    /** @brief Control the signed distance from the end-effector point to a target plane. */
    DistanceToPlane,
    /** @brief Control a line primitive attached to the end-effector. */
    Line,
    /** @brief Control a plane primitive attached to the end-effector. */
    Plane,
    /** @brief Control the full end-effector pose. */
    Pose,
    /** @brief Control only the end-effector orientation. */
    Rotation,
    /** @brief Control only the end-effector translation. */
    Translation
};

/**
 * @brief Abstract class that defines an interface to implement kinematic controllers for robots described by DQ_Kinematics.
 *
 * The controller stores a task-space control objective together with the primitives,
 * gains, damping terms, and stability-monitoring variables required by concrete
 * kinematic control laws. Subclasses are responsible for implementing the mapping
 * from task errors to reference joint velocities through either pure pseudoinverse,
 * constrained, or optimization-based formulations.
 *
 * @note This class remains abstract because subclasses must implement the setpoint
 * and tracking control laws.
 * @see DQ_PseudoinverseController, DQ_KinematicConstrainedController, DQ_ClassicQPController
 */
class DQ_KinematicController
{
protected:
    //Deprecated together with the raw pointer constructors but without the C++14 attribute as it is too noisy.
    /** @brief Legacy non-owning pointer to the robot model kept for backwards compatibility. */
    DQ_Kinematics* robot_;
    /** @brief Shared pointer to the robot model used by the smart-pointer constructors. */
    std::shared_ptr<DQ_Kinematics> robot_sptr_;
    /** @brief Current task-space objective handled by the controller. */
    ControlObjective control_objective_;
    /** @brief Primitive rigidly attached to the end-effector for line or plane control tasks. */
    DQ attached_primitive_;
    /** @brief Target primitive used by objectives that require convergence to a workspace primitive. */
    DQ target_primitive_;

    /** @brief Proportional gain used in the control law. */
    double gain_;
    /** @brief Isotropic damping used to regularize singular or ill-conditioned Jacobians. */
    double damping_;

    /** @brief True when the error evolution indicates convergence to a stable region. */
    bool system_reached_stable_region_;
    /** @brief Last joint-velocity control signal computed by the controller. */
    VectorXd last_control_signal_;
    /** @brief Last task-space error stored for stability verification. */
    VectorXd last_error_signal_;

    /** @brief Threshold on the variation of the task error used to detect a stable region. */
    double stability_threshold_;
    /** @brief Counter of consecutive iterations whose error variation is below the stability threshold. */
    int stability_counter_;
    /** @brief Number of consecutive stable iterations required to flag convergence to a stable region. */
    int stability_counter_max_;

    //For backwards compatibility reasons, to be removed
    /**
     * @brief Returns the active robot pointer regardless of the constructor used.
     *
     * @return Pointer to the associated robot model.
     */
    DQ_Kinematics* _get_robot_ptr() const;

    /**
     * @brief Returns the stored shared pointer to the robot model.
     *
     * @return Shared pointer to the associated robot model.
     * @throws std::runtime_error If the controller was not constructed with a shared pointer.
     */
    std::shared_ptr<DQ_Kinematics> _get_robot() const;

    //Deprecated
    /**
     * @brief Constructs a controller from a legacy raw robot pointer.
     *
     * @param robot Non-owning pointer to the robot kinematic model.
     */
    [[deprecated("Use the smart pointer version instead.")]]
    DQ_KinematicController(DQ_Kinematics* robot);
    /**
     * @brief Constructs a controller from a shared robot pointer.
     *
     * @param robot Shared pointer to the robot kinematic model.
     */
    DQ_KinematicController(const std::shared_ptr<DQ_Kinematics>& robot);
    /**
     * @brief Constructs a controller with default internal state.
     *
     * Concrete subclasses use this constructor to initialize gains, damping,
     * task selection, and stability-monitoring variables before binding a robot.
     */
    DQ_KinematicController();
public:

    /**
     * @brief Returns the current control objective.
     *
     * @return The configured control objective.
     */
    ControlObjective get_control_objective() const;

    /**
     * @brief Returns the task Jacobian associated with the current control objective.
     *
     * The returned matrix depends on the selected objective and can correspond to
     * pose, rotation, translation, distance, point-to-plane distance, line, or
     * plane kinematics.
     *
     * @param q Vector containing the current joint configurations of the robot.
     * @return The Jacobian associated with the current task variable.
     * @throws std::runtime_error If the number of joints is incompatible with the robot model,
     * if the control objective was not set, if a plane target was required but not configured,
     * or if the control objective is unknown.
     */
    MatrixXd get_jacobian(const VectorXd& q) const;

    /**
     * @brief Returns the current task variable associated with the control objective.
     *
     * The returned vector contains the task-space quantity regulated by the controller,
     * such as pose coordinates, translation, rotation, distance, line coordinates,
     * or plane coordinates.
     *
     * @param q Vector containing the current joint configurations of the robot.
     * @return The task variable corresponding to the configured objective.
     * @throws std::runtime_error If the number of joints is incompatible with the robot model,
     * if the control objective was not set, if a plane target was required but not configured,
     * or if the control objective is unknown.
     */
    VectorXd get_task_variable(const VectorXd& q) const;

    /**
     * @brief Returns the last task-space error signal computed by the controller.
     *
     * @return The last stored task-space error.
     */
    VectorXd get_last_error_signal() const;

    /**
     * @brief Verifies whether a control objective has been selected.
     *
     * @return True if the control objective is different from ControlObjective::None, and false otherwise.
     */
    bool is_set() const;

    /**
     * @brief Indicates whether the closed-loop system has reached a stable region.
     *
     * @return True if the error variation stayed below the stability threshold for the required number of iterations.
     */
    bool system_reached_stable_region() const;

    /**
     * @brief Sets the control objective.
     *
     * This method also resizes the internally stored error vector so that it matches
     * the dimension of the selected task variable.
     *
     * @param control_objective The desired control objective.
     */
    void set_control_objective(const ControlObjective& control_objective);

    /**
     * @brief Sets the controller gain.
     *
     * @param gain The proportional gain used in the control law.
     */
    void set_gain(const double& gain);
    /**
     * @brief Returns the controller gain.
     *
     * @return The current proportional gain.
     */
    double get_gain() const;

    /**
     * @brief Sets the isotropic damping used by singularity-robust controllers.
     *
     * @param damping The damping coefficient.
     */
    void set_damping(const double& damping);
    /**
     * @brief Returns the isotropic damping coefficient.
     *
     * @return The current damping coefficient.
     */
    double get_damping() const;

    /**
     * @brief Sets the threshold used to detect convergence to a stable region.
     *
     * @param threshold Maximum norm variation between consecutive task errors that is considered stable.
     */
    void set_stability_threshold(const double& threshold);

    /**
     * @brief Attaches a primitive to the end-effector for primitive-based objectives.
     *
     * For example, the primitive can represent a line or plane rigidly attached to the
     * end-effector and later used by ControlObjective::Line or ControlObjective::Plane.
     *
     * @param primitive Dual quaternion representation of the attached primitive.
     */
    void set_primitive_to_effector(const DQ& primitive);

    /**
     * @brief Sets the target primitive for primitive-based convergence tasks.
     *
     * @param primitive Dual quaternion representation of the target primitive.
     */
    void set_target_primitive(const DQ& primitive);

    /**
     * @brief Sets the number of consecutive stable iterations required to declare convergence.
     *
     * @param max Maximum value of the stability counter.
     */
    void set_stability_counter_max(const int& max);

    /**
     * @brief Resets the stability counter and clears the stable-region flag.
     */
    void reset_stability_counter();

    //Virtual
    /**
     * @brief Virtual destructor.
     */
    virtual ~DQ_KinematicController()=default;
    /**
     * @brief Computes the control signal for a setpoint task.
     *
     * Pure virtual interface contract implemented by concrete kinematic controllers.
     *
     * @param q Vector containing the current joint configurations of the robot.
     * @param task_reference Vector containing the desired value for the chosen control task.
     * @return The reference joint velocities.
     */
    virtual VectorXd compute_setpoint_control_signal(const VectorXd& q, const VectorXd& task_reference)=0;
    /**
     * @brief Computes the reference joint velocities for a tracking task with feedforward.
     *
     * Pure virtual interface contract implemented by concrete kinematic controllers.
     *
     * @param q Vector containing the current joint configurations of the robot.
     * @param task_reference Vector containing the desired value for the chosen control task.
     * @param feed_forward Time derivative of the task reference expressed in task space.
     * @return The reference joint velocities.
     */
    virtual VectorXd compute_tracking_control_signal(const VectorXd& q, const VectorXd& task_reference, const VectorXd& feed_forward)=0;
    /**
     * @brief Updates the internal stability indicators using the current task error.
     *
     * The system is considered to have reached a stable region when the variation of
     * the task error remains below the configured threshold for a sufficient number of
     * consecutive iterations.
     *
     * @param task_error Current task-space error.
     */
    virtual void verify_stability(const VectorXd& task_error);

};

}

