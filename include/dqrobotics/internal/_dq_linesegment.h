#pragma once
#pragma message("_dq_linesegment.h is an internal DQRobotics header and its ABI/API is not stable.")
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
#include <tuple>

namespace DQ_robotics
{


namespace internal
{

/**
 * @brief Internal routines for closest-feature queries between line segments.
 *
 * This class supports the line-segment distance algorithms used inside the C++
 * implementation of DQ Robotics. It is declared in the internal namespace and
 * is not part of the stable public API.
 *
 * @internal
 */
class LineSegment
{
public:
    /**
     * @brief Identifies which primitive of a line segment participates in a closest pair.
     */
    enum class Element{
        /** @brief The supporting infinite line of the segment. */
        Line,
        /** @brief The first endpoint of the segment. */
        P1,
        /** @brief The second endpoint of the segment. */
        P2
    };

    /**
     * @brief Groups the supporting line and the two endpoints of a line segment.
     */
    using Primitives = std::tuple<DQ,DQ,DQ>;

    /**
     * @brief Stores the closest primitive selected from each of two line segments.
     */
    using ClosestElements = std::tuple<Element,Element>;

    /**
     * @brief Stores a closest-element pair together with its squared distance.
     */
    using ClosestElementsAndDistance = std::tuple<ClosestElements, double>;

    /**
     * @brief Determines the closest primitives between two line segments.
     *
     * The algorithm compares admissible line-line, line-endpoint, and
     * endpoint-endpoint candidates, discarding infeasible line-to-point cases,
     * and returns the valid pair with minimum squared distance.
     *
     * @param line_1_primitives A tuple `(line, point_1, point_2)` describing the first segment.
     * @param line_2_primitives A tuple `(line, point_1, point_2)` describing the second segment.
     * @return A tuple containing the closest primitive pair and the corresponding squared distance.
     */
    static ClosestElementsAndDistance closest_elements_between_line_segments(const Primitives& line_1_primitives,
                                                                             const Primitives& line_2_primitives);


    /**
     * @brief Tests whether a point lies strictly inside a line segment.
     *
     * This check compares the squared distances from the point to each endpoint
     * against the segment squared length. It assumes the point already belongs to
     * the supporting line; the collinearity test is not performed here.
     *
     * @param point The point to be tested.
     * @param line_1_primitives A tuple `(line, point_1, point_2)` describing the segment.
     * @return `true` if the point lies strictly between the endpoints and `false` otherwise.
     * @note Endpoints are considered outside because strict inequalities are used.
     */
    static bool is_inside_line_segment(const DQ &point, const Primitives &line_1_primitives);

    /**
     * @brief Converts an Element enumerator to its string representation.
     *
     * @param e The enumerator to be converted.
     * @return The string representation of `e`.
     * @throws std::runtime_error If `e` does not match a known enumerator.
     */
    static std::string to_string(const Element& e);


private:

    /**
     * @brief Selects the best valid closest-pair candidate seen so far.
     *
     * Invalid candidates are represented with `NaN` distances and are ignored.
     * When both candidates are valid, the one with smaller squared distance is
     * returned.
     *
     * @param current The current best result.
     * @param candidate The new candidate to be compared against `current`.
     * @return The preferred result after the comparison.
     */
    static ClosestElementsAndDistance _update_closest_pair(
            const ClosestElementsAndDistance& current,
            const ClosestElementsAndDistance& candidate);

    /**
     * @brief Evaluates a line-to-point candidate for the line-segment search.
     *
     * The method projects the point onto the supporting line of the segment. If
     * the projection lies inside the segment, it returns the squared point-to-line
     * distance; otherwise, it returns `NaN` to mark the candidate as infeasible.
     *
     * @param line_segment A tuple `(line, point_1, point_2)` describing the segment.
     * @param point The point from the other segment.
     * @return The squared point-to-line distance for a feasible candidate, or `NaN` otherwise.
     */
    static double _line_to_point_feasibility_and_distance(
            const Primitives& line_segment,
            const DQ& point);
};

}

}
