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

#ifndef DQ_ROBOT_MODELLING_DQ_KINEMATICS_H
#define DQ_ROBOT_MODELLING_DQ_KINEMATICS_H

#include<dqrobotics/DQ.h>

namespace DQ_robotics
{
/**
 * @brief Abstract class that defines an interface to implement robot kinematics.
 *
 * DQ_Kinematics is the common base class for robot models in DQ Robotics.
 * It stores the reference and base frames, the dimension of the configuration
 * space, and static operations that derive task-space Jacobians from a pose
 * Jacobian represented in dual quaternion form.
 *
 * @note The methods declared with = 0 are pure virtual interface contracts and
 * must be implemented by subclasses. The virtual overloads without an explicit
 * link index are concrete convenience wrappers that default to the last link.
 *
 * @see DQ_SerialManipulator, DQ_MobileBase, DQ_CooperativeDualTaskSpace
 */
class DQ_Kinematics
{
protected:
    /** @brief Name assigned to the kinematic model. */
    std::string name_;

    /** @brief Reference frame used by fkm() and pose_jacobian() computations. */
    DQ reference_frame_;
    /** @brief Physical placement of the robot base in the workspace. */
    DQ base_frame_;

    /** @brief Configuration vector associated with the model state. */
    VectorXd q_;
    /** @brief Dimension of the configuration space. */
    int dim_configuration_space_;

    /** @brief Lower joint-position limits. */
    VectorXd lower_q_limit_;
    /** @brief Upper joint-position limits. */
    VectorXd upper_q_limit_;
    /** @brief Lower joint-velocity limits. */
    VectorXd lower_q_dot_limit_;
    /** @brief Upper joint-velocity limits. */
    VectorXd upper_q_dot_limit_;

    /**
     * @brief Checks whether a link index is valid for this model.
     *
     * @param to_ith_link Index of the link to be checked.
     * @throws std::runtime_error If @p to_ith_link is outside the valid range.
     */
    void _check_to_ith_link(const int& to_ith_link) const;
    /**
     * @brief Checks whether a configuration vector has the correct dimension.
     *
     * @param q_vec Vector to be validated.
     * @throws std::runtime_error If @p q_vec does not match the configuration-space dimension.
     */
    void _check_q_vec(const VectorXd& q_vec) const;

    //Constructor
    /** @brief Constructs a kinematic model with identity reference and base frames. */
    DQ_Kinematics();
public:
    //Virtual destructor
    /** @brief Virtual destructor. */
    virtual ~DQ_Kinematics() = default;

    //Concrete methods
    /**
     * @brief Sets the reference frame used by the forward kinematics and Jacobian methods.
     *
     * @param get_reference_frame Unit dual quaternion representing the reference frame.
     * @throws std::runtime_error If @p get_reference_frame is not a unit dual quaternion.
     */
    void set_reference_frame(const DQ& get_reference_frame);
    /**
     * @brief Returns the current reference frame.
     *
     * @return The reference frame as a unit dual quaternion.
     */
    DQ   get_reference_frame() const;
    /**
     * @brief Sets the physical base frame of the robot in the workspace.
     *
     * The base frame determines the physical placement of the robot and does not
     * need to coincide with the reference frame used for calculations.
     *
     * @param get_base_frame Unit dual quaternion representing the base frame.
     * @throws std::runtime_error If @p get_base_frame is not a unit dual quaternion.
     */
    void set_base_frame(const DQ& get_base_frame);
    /**
     * @brief Returns the current base frame.
     *
     * @return The base frame as a unit dual quaternion.
     */
    DQ   get_base_frame() const;
    /**
     * @brief Sets the name of the kinematic model.
     *
     * @param get_name Name to be assigned to the model.
     */
    void set_name(const std::string& get_name);
    /**
     * @brief Returns the model name.
     *
     * @return The current model name.
     */
    std::string get_name() const;

    //PURE virtual methods
    /**
     * @brief Computes the forward kinematics up to the last link.
     *
     * This is a pure virtual interface contract and must be implemented by subclasses.
     *
     * @param joint_configurations Vector containing the robot joint configurations.
     * @return The pose of the last link, represented as a unit dual quaternion.
     */
    virtual DQ fkm                (const VectorXd& joint_configurations) const = 0;
    /**
     * @brief Computes the forward kinematics up to a given link.
     *
     * This is a pure virtual interface contract and must be implemented by subclasses.
     *
     * @param joint_configurations Vector containing the robot joint configurations.
     * @param to_ith_link Index of the last link to be accounted for in the computation.
     * @return The pose of the reference frame attached to the ith link, represented as a unit dual quaternion.
     */
    virtual DQ fkm                (const VectorXd& joint_configurations, const int& to_ith_link) const = 0;
    /**
     * @brief Computes the pose Jacobian up to a given link.
     *
     * This is a pure virtual interface contract and must be implemented by subclasses.
     *
     * @param joint_configurations Vector containing the robot joint configurations.
     * @param to_ith_link Index of the last link to be accounted for in the computation.
     * @return The pose Jacobian that satisfies vec8(pose_dot) = J * q_dot.
     */
    virtual MatrixXd pose_jacobian(const VectorXd& joint_configurations, const int& to_ith_link) const = 0;
    /**
     * @brief Computes the time derivative of the pose Jacobian up to a given link.
     *
     * This is a pure virtual interface contract and must be implemented by subclasses.
     *
     * @param q Vector containing the robot joint configurations.
     * @param q_dot Vector containing the robot joint velocities.
     * @param to_ith_link Index of the last link to be accounted for in the computation.
     * @return The Jacobian derivative that satisfies vec8(pose_ddot) = J_dot * q_dot + J * q_ddot.
     */
    virtual MatrixXd pose_jacobian_derivative(const VectorXd& q,
                                              const VectorXd& q_dot,
                                              const int& to_ith_link) const = 0;
    //Virtual methods
    /**
     * @brief Computes the pose Jacobian up to the last link.
     *
     * This concrete overload delegates to pose_jacobian(joint_configurations, get_dim_configuration_space() - 1).
     *
     * @param joint_configurations Vector containing the robot joint configurations.
     * @return The pose Jacobian that satisfies vec8(pose_dot) = J * q_dot.
     */
    virtual MatrixXd pose_jacobian (const VectorXd& joint_configurations) const;
    /**
     * @brief Computes the time derivative of the pose Jacobian up to the last link.
     *
     * This concrete overload delegates to pose_jacobian_derivative(q, q_dot, get_dim_configuration_space() - 1).
     *
     * @param q Vector containing the robot joint configurations.
     * @param q_dot Vector containing the robot joint velocities.
     * @return The Jacobian derivative that satisfies vec8(pose_ddot) = J_dot * q_dot + J * q_ddot.
     */
    virtual MatrixXd pose_jacobian_derivative(const VectorXd& q,
                                              const VectorXd& q_dot) const;
    /**
     * @brief Returns the dimension of the configuration space.
     *
     * @return Number of generalized coordinates of the model.
     */
    virtual int get_dim_configuration_space() const;

    //Static methods
    /**
     * @brief Computes the squared-distance Jacobian from a pose Jacobian.
     *
     * @param pose_jacobian Pose Jacobian associated with @p pose.
     * @param pose Pose associated with @p pose_jacobian.
     * @return The Jacobian of the squared distance from the pose origin to the reference-frame origin.
     */
    static MatrixXd distance_jacobian    (const MatrixXd& pose_jacobian, const DQ& pose);
    /**
     * @brief Computes the translation Jacobian from a pose Jacobian.
     *
     * @param pose_jacobian Pose Jacobian associated with @p pose.
     * @param pose Pose associated with @p pose_jacobian.
     * @return The Jacobian that satisfies vec4(translation_dot) = J * q_dot.
     */
    static MatrixXd translation_jacobian (const MatrixXd& pose_jacobian, const DQ& pose);
    /**
     * @brief Extracts the rotation Jacobian from a pose Jacobian.
     *
     * @param pose_jacobian Pose Jacobian in dual quaternion form.
     * @return The Jacobian that satisfies vec4(rotation_dot) = J * q_dot.
     */
    static MatrixXd rotation_jacobian    (const MatrixXd& pose_jacobian);
    /**
     * @brief Computes the Jacobian of a line rigidly attached to a pose.
     *
     * @param pose_jacobian Pose Jacobian associated with @p pose.
     * @param pose Pose associated with @p pose_jacobian.
     * @param line_direction Line direction expressed in the local frame of @p pose.
     * @return The line Jacobian of the line obtained by transforming @p line_direction with @p pose.
     */
    static MatrixXd line_jacobian        (const MatrixXd& pose_jacobian, const DQ& pose, const DQ& line_direction);
    /**
     * @brief Computes the Jacobian of a plane rigidly attached to a pose.
     *
     * @param pose_jacobian Pose Jacobian associated with @p pose.
     * @param pose Pose associated with @p pose_jacobian.
     * @param plane_normal Plane normal expressed in the local frame of @p pose.
     * @return The plane Jacobian of the plane obtained by transforming @p plane_normal with @p pose.
     */
    static MatrixXd plane_jacobian       (const MatrixXd& pose_jacobian, const DQ& pose, const DQ& plane_normal);
    /**
     * @brief Extracts the rotation-Jacobian derivative from a pose-Jacobian derivative.
     *
     * @param pose_jacobian_derivative Pose-Jacobian derivative in dual quaternion form.
     * @return The derivative of the rotation Jacobian.
     */
    static MatrixXd rotation_jacobian_derivative    (const MatrixXd& pose_jacobian_derivative);
    /**
     * @brief Computes the time derivative of the translation Jacobian.
     *
     * @param pose_jacobian Pose Jacobian associated with @p pose.
     * @param pose_jacobian_derivative Time derivative of @p pose_jacobian.
     * @param pose Pose associated with @p pose_jacobian.
     * @param q_dot Vector containing the robot configuration velocities.
     * @return The derivative of the translation Jacobian.
     */
    static MatrixXd translation_jacobian_derivative (const MatrixXd& pose_jacobian,
                                                     const MatrixXd& pose_jacobian_derivative,
                                                     const DQ& pose,
                                                     const VectorXd &q_dot);
    /**
     * @brief Computes the time derivative of the squared-distance Jacobian.
     *
     * @param pose_jacobian Pose Jacobian associated with @p pose.
     * @param pose_jacobian_derivative Time derivative of @p pose_jacobian.
     * @param pose Pose associated with @p pose_jacobian.
     * @param q_dot Vector containing the robot configuration velocities.
     * @return The derivative of the squared-distance Jacobian.
     */
    static MatrixXd distance_jacobian_derivative    (const MatrixXd& pose_jacobian,
                                                     const MatrixXd& pose_jacobian_derivative,
                                                     const DQ& pose,
                                                     const VectorXd &q_dot);
    /**
     * @brief Computes the time derivative of a plane Jacobian.
     *
     * @param pose_jacobian Pose Jacobian associated with @p pose.
     * @param pose_jacobian_derivative Time derivative of @p pose_jacobian.
     * @param pose Pose associated with @p pose_jacobian.
     * @param plane_normal Plane normal expressed in the local frame of @p pose.
     * @param q_dot Vector containing the robot configuration velocities.
     * @return The derivative of the plane Jacobian.
     */
    static MatrixXd plane_jacobian_derivative       (const MatrixXd& pose_jacobian,
                                                     const MatrixXd& pose_jacobian_derivative,
                                                     const DQ& pose,
                                                     const DQ& plane_normal,
                                                     const VectorXd &q_dot);
    /**
     * @brief Computes the time derivative of a line Jacobian.
     *
     * @param pose_jacobian Pose Jacobian associated with @p pose.
     * @param pose_jacobian_derivative Time derivative of @p pose_jacobian.
     * @param pose Pose associated with @p pose_jacobian.
     * @param line_direction Line direction expressed in the local frame of @p pose.
     * @param q_dot Vector containing the robot configuration velocities.
     * @return The derivative of the line Jacobian.
     */
    static MatrixXd line_jacobian_derivative        (const MatrixXd& pose_jacobian,
                                                     const MatrixXd& pose_jacobian_derivative,
                                                     const DQ& pose,
                                                     const DQ& line_direction,
                                                     const VectorXd &q_dot);
    /**
     * @brief Computes the squared point-to-point distance Jacobian.
     *
     * @param translation_jacobian Translation Jacobian of the robot point.
     * @param robot_point Point rigidly attached to the robot, represented as a pure quaternion.
     * @param workspace_point Workspace point, represented as a pure quaternion.
     * @return The squared point-to-point distance Jacobian.
     * @throws std::range_error If either point is not a pure quaternion.
     */
    static MatrixXd point_to_point_distance_jacobian(const MatrixXd& translation_jacobian, const DQ& robot_point, const DQ& workspace_point);
    /**
     * @brief Computes the residual term of the squared point-to-point distance dynamics.
     *
     * @param robot_point Point rigidly attached to the robot, represented as a pure quaternion.
     * @param workspace_point Workspace point, represented as a pure quaternion.
     * @param workspace_point_derivative Time derivative of the workspace point.
     * @return The residual term associated with the workspace-point motion.
     * @throws std::range_error If @p robot_point or @p workspace_point is not a pure quaternion.
     */
    static double   point_to_point_residual         (const DQ& robot_point, const DQ& workspace_point, const DQ& workspace_point_derivative);
    /**
     * @brief Computes the squared point-to-line distance Jacobian.
     *
     * @param translation_jacobian Translation Jacobian of the robot point.
     * @param robot_point Point rigidly attached to the robot, represented as a pure quaternion.
     * @param workspace_line Workspace line.
     * @return The squared point-to-line distance Jacobian.
     * @throws std::range_error If @p robot_point is not a pure quaternion or @p workspace_line is not a line.
     */
    static MatrixXd point_to_line_distance_jacobian (const MatrixXd& translation_jacobian, const DQ& robot_point, const DQ& workspace_line);
    /**
     * @brief Computes the residual term of the squared point-to-line distance dynamics.
     *
     * @param robot_point Point rigidly attached to the robot, represented as a pure quaternion.
     * @param workspace_line Workspace line.
     * @param workspace_line_derivative Time derivative of the workspace line.
     * @return The residual term associated with the workspace-line motion.
     * @throws std::range_error If @p robot_point is not a pure quaternion or @p workspace_line is not a line.
     */
    static double   point_to_line_residual          (const DQ& robot_point, const DQ& workspace_line, const DQ& workspace_line_derivative);
    /**
     * @brief Computes the squared point-to-plane distance Jacobian.
     *
     * @param translation_jacobian Translation Jacobian of the robot point.
     * @param robot_point Point rigidly attached to the robot, represented as a pure quaternion.
     * @param workspace_plane Workspace plane.
     * @return The squared point-to-plane distance Jacobian.
     * @throws std::range_error If @p robot_point is not a pure quaternion or @p workspace_plane is not a plane.
     */
    static MatrixXd point_to_plane_distance_jacobian(const MatrixXd& translation_jacobian, const DQ& robot_point, const DQ& workspace_plane);
    /**
     * @brief Computes the residual term of the squared point-to-plane distance dynamics.
     *
     * @param translation Translation of the robot point, represented as a pure quaternion.
     * @param plane_derivative Time derivative of the workspace plane.
     * @return The residual term associated with the workspace-plane motion.
     * @throws std::range_error If @p translation is not a pure quaternion.
     */
    static double   point_to_plane_residual         (const DQ& translation, const DQ& plane_derivative);
    /**
     * @brief Computes the squared line-to-point distance Jacobian.
     *
     * @param line_jacobian Line Jacobian of the robot line.
     * @param robot_line Line rigidly attached to the robot.
     * @param workspace_point Workspace point, represented as a pure quaternion.
     * @return The squared line-to-point distance Jacobian.
     * @throws std::range_error If @p robot_line is not a line or @p workspace_point is not a pure quaternion.
     */
    static MatrixXd line_to_point_distance_jacobian (const MatrixXd& line_jacobian, const DQ& robot_line, const DQ& workspace_point);
    /**
     * @brief Computes the residual term of the squared line-to-point distance dynamics.
     *
     * @param robot_line Line rigidly attached to the robot.
     * @param workspace_point Workspace point, represented as a pure quaternion.
     * @param workspace_point_derivative Time derivative of the workspace point.
     * @return The residual term associated with the workspace-point motion.
     * @throws std::range_error If @p robot_line is not a line or @p workspace_point is not a pure quaternion.
     */
    static double   line_to_point_residual          (const DQ& robot_line, const DQ& workspace_point, const DQ& workspace_point_derivative);
    /**
     * @brief Computes the squared line-to-line distance Jacobian.
     *
     * @param line_jacobian Line Jacobian of the robot line.
     * @param robot_line Line rigidly attached to the robot.
     * @param workspace_line Workspace line.
     * @return The squared line-to-line distance Jacobian.
     * @throws std::range_error If @p robot_line or @p workspace_line is not a line.
     */
    static MatrixXd line_to_line_distance_jacobian  (const MatrixXd& line_jacobian, const DQ& robot_line, const DQ& workspace_line);
    /**
     * @brief Computes the residual term of the squared line-to-line distance dynamics.
     *
     * @param robot_line Line rigidly attached to the robot.
     * @param workspace_line Workspace line.
     * @param workspace_line_derivative Time derivative of the workspace line.
     * @return The residual term associated with the workspace-line motion.
     * @throws std::range_error If @p robot_line or @p workspace_line is not a line.
     */
    static double   line_to_line_residual           (const DQ& robot_line, const DQ& workspace_line, const DQ& workspace_line_derivative);
    /**
     * @brief Computes the squared plane-to-point distance Jacobian.
     *
     * @param plane_jacobian Plane Jacobian of the robot plane.
     * @param workspace_point Workspace point, represented as a pure quaternion.
     * @return The squared plane-to-point distance Jacobian.
     * @throws std::range_error If @p workspace_point is not a pure quaternion.
     */
    static MatrixXd plane_to_point_distance_jacobian(const MatrixXd& plane_jacobian, const DQ& workspace_point);
    /**
     * @brief Computes the residual term of the squared plane-to-point distance dynamics.
     *
     * @param robot_plane Plane rigidly attached to the robot.
     * @param workspace_point_derivative Time derivative of the workspace point.
     * @return The residual term associated with the workspace-point motion.
     * @throws std::range_error If @p workspace_point_derivative is not a pure quaternion.
     */
    static double   plane_to_point_residual         (const DQ& robot_plane, const DQ& workspace_point_derivative);
    /**
     * @brief Computes the Jacobian of the line-to-line angle objective.
     *
     * @param line_jacobian Line Jacobian of the robot line.
     * @param robot_line Line rigidly attached to the robot.
     * @param workspace_line Workspace line.
     * @return The Jacobian associated with the objective f(phi) = dot(robot_line - workspace_line, robot_line - workspace_line).
     * @throws std::range_error If @p robot_line or @p workspace_line is not a line.
     */
    static MatrixXd line_to_line_angle_jacobian     (const MatrixXd& line_jacobian, const DQ& robot_line, const DQ& workspace_line);
    /**
     * @brief Computes the residual term of the line-to-line angle objective.
     *
     * @param robot_line Line rigidly attached to the robot.
     * @param workspace_line Workspace line.
     * @param workspace_line_derivative Time derivative of the workspace line.
     * @return The residual term associated with workspace-line motion.
     * @throws std::range_error If @p robot_line or @p workspace_line is not a line.
     */
    static double   line_to_line_angle_residual     (const DQ& robot_line, const DQ& workspace_line, const DQ& workspace_line_derivative);

    /**
     * @brief Computes a squared-distance Jacobian between two line segments.
     *
     * The method selects the appropriate point-to-point, point-to-line, or
     * line-to-line Jacobian according to the closest elements of the two
     * segments, following the active-constraints formulation used in DQ Robotics.
     *
     * @param line_jacobian Line Jacobian of @p robot_line.
     * @param robot_point_1_translation_jacobian Translation Jacobian of the first endpoint of the robot segment.
     * @param robot_point_2_translation_jacobian Translation Jacobian of the second endpoint of the robot segment.
     * @param robot_line Line containing the robot segment.
     * @param robot_point_1 First endpoint of the robot segment.
     * @param robot_point_2 Second endpoint of the robot segment.
     * @param workspace_line Line containing the workspace segment.
     * @param workspace_point_1 First endpoint of the workspace segment.
     * @param workspace_point_2 Second endpoint of the workspace segment.
     * @return The squared-distance Jacobian between the two segments.
     * @throws std::runtime_error If the provided line-segment data is inconsistent or an unexpected closest-element case is reached.
     */
    static MatrixXd line_segment_to_line_segment_distance_jacobian(const MatrixXd& line_jacobian,
                                                                   const MatrixXd& robot_point_1_translation_jacobian,
                                                                   const MatrixXd& robot_point_2_translation_jacobian,
                                                                   const DQ& robot_line,
                                                                   const DQ& robot_point_1,
                                                                   const DQ& robot_point_2,
                                                                   const DQ& workspace_line,
                                                                   const DQ& workspace_point_1,
                                                                   const DQ& workspace_point_2);

};
}

#endif
