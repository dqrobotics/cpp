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


#include<dqrobotics/DQ.h>
#include<tuple>

namespace DQ_robotics
{

/**
 * @brief Provides geometric operations for points, lines, planes, and line segments.
 *
 * This class groups static methods that evaluate distances, angles, projections,
 * and closest-point relations directly from the dual-quaternion geometric
 * primitives used throughout DQ Robotics.
 */
class DQ_Geometry
{
public:
    /**
     * @brief Computes the squared distance between two points represented as pure quaternions.
     *
     * @param point1 The pure quaternion representing the first point.
     * @param point2 The pure quaternion representing the second point.
     * @return The squared Euclidean distance between the two points.
     * @throws std::range_error If either input is not a pure quaternion.
     * @see point_to_line_squared_distance, point_to_plane_distance
     */
    static double point_to_point_squared_distance(const DQ& point1, const DQ& point2);

    /**
     * @brief Computes the squared distance between a point and a line.
     *
     * The point must be given as a pure quaternion and the line as a unit-norm
     * pure dual quaternion.
     *
     * @param point The pure quaternion representing the point.
     * @param line The dual quaternion representing the line.
     * @return The squared Euclidean distance between the point and the line.
     * @throws std::range_error If `point` is not a pure quaternion or `line` is not a line.
     * @see point_to_point_squared_distance, point_projected_in_line
     */
    static double point_to_line_squared_distance(const DQ& point, const DQ& line);

    /**
     * @brief Computes the signed distance between a point and a plane.
     *
     * The point must be represented as a pure quaternion, and the plane must be
     * represented as a unit dual quaternion with pure primary part and real dual
     * part.
     *
     * @param point The pure quaternion representing the point.
     * @param plane The dual quaternion representing the plane.
     * @return The signed distance from the point to the plane.
     * @note The sign is determined by the orientation of the plane normal.
     * @throws std::range_error If `point` is not a pure quaternion or `plane` is not a plane.
     * @see point_to_point_squared_distance, point_to_line_squared_distance
     */
    static double point_to_plane_distance(const DQ& point, const DQ& plane);

    /**
     * @brief Computes the squared distance between two lines.
     *
     * For non-parallel lines, the result follows the ratio between the norm of
     * the dual part of their dot product and the norm of the primary part of
     * their cross product. For parallel lines, the implementation falls back to
     * the dual part of the line cross product.
     *
     * @param line1 The first dual quaternion line.
     * @param line2 The second dual quaternion line.
     * @return The squared Euclidean distance between the two lines.
     * @note This method returns a squared distance, not the distance itself.
     * @throws std::range_error If either input is not a line.
     * @see line_to_line_angle, closest_points_between_lines
     */
    static double line_to_line_squared_distance(const DQ& line1, const DQ& line2);

    /**
     * @brief Computes the angle between two lines.
     *
     * The angle is obtained from the primary part of the dot product between the
     * two dual-quaternion line representations.
     *
     * @param line1 The first dual quaternion line.
     * @param line2 The second dual quaternion line.
     * @return The angle between the two lines in radians.
     * @throws std::range_error If either input is not a line.
     * @see line_to_line_squared_distance
     */
    static double line_to_line_angle(const DQ& line1, const DQ& line2);

    /**
     * @brief Projects a point onto a line.
     *
     * @param point The pure quaternion representing the point to be projected.
     * @param line The dual quaternion representing the line.
     * @return The orthogonal projection of the point onto the line as a pure quaternion.
     * @see point_to_line_squared_distance, closest_points_between_lines
     */
    static DQ point_projected_in_line(const DQ& point, const DQ& line);

    /**
     * @brief Computes the closest points between two lines.
     *
     * The returned tuple contains one point on each line. The closed-form
     * expression assumes that the supporting lines are not parallel.
     *
     * @param line1 The first dual quaternion line.
     * @param line2 The second dual quaternion line.
     * @return A tuple containing the closest point on `line1` and the closest point on `line2`.
     * @note Parallel lines do not define a unique closest-point pair for this formulation.
     * @throws std::runtime_error If either input is not a line.
     * @see line_to_line_squared_distance, point_projected_in_line
     */
    static std::tuple<DQ,DQ> closest_points_between_lines(const DQ& line1, const DQ& line2);

    /**
     * @brief Checks whether a line and two endpoints define a valid line segment.
     *
     * The method verifies that the supporting primitive is a line, that both
     * endpoints are pure quaternions, and that both endpoints lie on the line up
     * to the supplied threshold.
     *
     * @param line The dual quaternion representing the supporting line.
     * @param line_point_1 The first endpoint of the segment.
     * @param line_point_2 The second endpoint of the segment.
     * @param threshold The tolerance used to test whether each endpoint lies on the line.
     * @return `true` if the inputs define a valid line segment and `false` otherwise.
     * @note The tolerance is applied only to the point-on-line consistency test.
     * @see closest_points_between_line_segments, line_segment_to_line_segment_squared_distance
     */
    static bool is_line_segment(const DQ& line,
                                const DQ& line_point_1,
                                const DQ& line_point_2,
                                const double& threshold=DQ_threshold);

    /**
     * @brief Computes the closest points between two line segments.
     *
     * The implementation evaluates the relevant line-line, line-endpoint, and
     * endpoint-endpoint candidates and returns the pair that minimizes the
     * squared distance.
     *
     * @param line_1 The first supporting line.
     * @param line_1_point_1 The first endpoint of the first segment.
     * @param line_1_point_2 The second endpoint of the first segment.
     * @param line_2 The second supporting line.
     * @param line_2_point_1 The first endpoint of the second segment.
     * @param line_2_point_2 The second endpoint of the second segment.
     * @return A tuple containing the closest point on the first segment and the closest point on the second segment.
     * @note A unique closest-point pair is only well defined when the supporting lines are not parallel.
     * @throws std::runtime_error If either input triple does not define a valid line segment or if no unique pair can be resolved.
     * @see closest_points_between_lines, line_segment_to_line_segment_squared_distance
     */
    static std::tuple<DQ,DQ> closest_points_between_line_segments(const DQ& line_1,
                                                                  const DQ& line_1_point_1,
                                                                  const DQ& line_1_point_2,
                                                                  const DQ& line_2,
                                                                  const DQ& line_2_point_1,
                                                                  const DQ& line_2_point_2);

    /**
     * @brief Computes the squared distance between two line segments.
     *
     * For non-parallel supporting lines, the result is the minimum squared
     * distance over the same candidate set used by
     * closest_points_between_line_segments(). For parallel supporting lines, the
     * method returns the squared distance between the corresponding infinite
     * lines.
     *
     * @param line_1 The first supporting line.
     * @param line_1_point_1 The first endpoint of the first segment.
     * @param line_1_point_2 The second endpoint of the first segment.
     * @param line_2 The second supporting line.
     * @param line_2_point_1 The first endpoint of the second segment.
     * @param line_2_point_2 The second endpoint of the second segment.
     * @return The squared Euclidean distance between the two line segments.
     * @note This method returns a squared distance, not the distance itself.
     * @throws std::runtime_error If either input triple does not define a valid line segment.
     * @see closest_points_between_line_segments, line_to_line_squared_distance
     */
    static double line_segment_to_line_segment_squared_distance(const DQ& line_1,
                                                                const DQ& line_1_point_1,
                                                                const DQ& line_1_point_2,
                                                                const DQ& line_2,
                                                                const DQ& line_2_point_1,
                                                                const DQ& line_2_point_2);

};

}


