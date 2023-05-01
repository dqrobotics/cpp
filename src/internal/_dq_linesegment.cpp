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
#include <dqrobotics/internal/_dq_linesegment.h>
#include <dqrobotics/utils/DQ_Geometry.h>
#include <vector>


namespace DQ_robotics
{
namespace internal
{

#ifdef DQ_ROBOTICS_INTERNAL_DQ_LINESEGMENT_VERBOSE
constexpr bool _verbose = true;
#else
constexpr bool _verbose = false;
#endif

bool LineSegment::is_inside_line_segment(const DQ& point,
                                         const Primitives& line_primitives)
{
    const auto& p1 = std::get<1>(line_primitives);
    const auto& p2 = std::get<2>(line_primitives);

    const double& D_p1 = DQ_Geometry::point_to_point_squared_distance(point,p1);
    const double& D_p2 = DQ_Geometry::point_to_point_squared_distance(point,p2);

    const double& segment_size = DQ_Geometry::point_to_point_squared_distance(p1,p2);

    return ((D_p1 < segment_size) && (D_p2 < segment_size));
}

std::string LineSegment::to_string(const Element &e)
{
    switch(e)
    {
    case Element::Line:
        return std::string("Line");
    case Element::P1:
        return std::string("P1");
    case Element::P2:
        return std::string("P2");
    default:
        throw std::runtime_error("Unknown case.");
    }
}

LineSegment::ClosestElementsAndDistance LineSegment::_update_closest_pair(const ClosestElementsAndDistance &current, const ClosestElementsAndDistance &candidate)
{
    if(std::isnan(std::get<1>(candidate)))
        return current;
    if(std::isnan(std::get<1>(current)))
    {
        if(_verbose)
        {
            std::cout << "Update to candidate {"
                      << to_string(std::get<0>(std::get<0>(candidate)))
                      << ","
                      << to_string(std::get<1>(std::get<0>(candidate)))
                      << "}"
                      << std::endl;
        }
        return candidate;
    }
    else
    {
        if(_verbose)
        {
            std::cout << "D = "
                      << std::to_string(std::get<1>(current))
                      << " < "
                      << std::to_string(std::get<1>(candidate))
                      << std::endl;
        }
        if(std::get<1>(candidate) < std::get<1>(current))
        {
            if(_verbose)
            {
                std::cout << "Update to candidate {"
                          << to_string(std::get<0>(std::get<0>(candidate)))
                          << ","
                          << to_string(std::get<1>(std::get<0>(candidate)))
                          << "}"
                          << std::endl;
            }
            return candidate;
        }
        else
        {
            if(_verbose)
            {
                std::cout << "Keep current {"
                          << to_string(std::get<0>(std::get<0>(candidate)))
                          << ","
                          << to_string(std::get<1>(std::get<0>(candidate)))
                          << "}"
                          << std::endl;
            }
            return current;
        }
    }
}

double LineSegment::_line_to_point_feasibility_and_distance(const Primitives &line_segment, const DQ &point)
{
    bool is_possible = is_inside_line_segment(DQ_Geometry::point_projected_in_line(point,std::get<0>(line_segment)),
                                              line_segment);
    if(is_possible)
    {
        return DQ_Geometry::point_to_line_squared_distance(point,
                                                           std::get<0>(line_segment)
                                                           );
    }
    return std::nan("");
}

/**
 * @brief LineSegment::closest_elements_between_line_segments.
 * Obtain the closest elements between two line segments and their distance.
 * A simplified logic is as follows:
 * 1. Obtain the closest points between lines. If those points rest in
 * the line segments, store the distance in D_i. Otherwise, D_i is invalid.
 * 2. Obtain projections of points in the lines. If those projections rest
 * in the line segments, store the distance in D_i. Otherwise, D_i is invalid.
 * 3. Obtain the distance between the relevant line segment endpoints and
 * store them in D_i.
 * The closest elements will be defined by the elements with smallest D_i
 * which is valid.
 * @param line_1_primitives a tuple{line,point,point} describing the line segment.
 * @param line_2_primitives a tuple{line,point,point} describing the line segment.
 * @return a tuple {{Element,Element},squared_distance}.
 */
LineSegment::ClosestElementsAndDistance
LineSegment::closest_elements_between_line_segments(const Primitives& line_1_primitives,
                                                    const Primitives& line_2_primitives)
{
    const auto& line_1 = std::get<0>(line_1_primitives);
    const auto& line_2 = std::get<0>(line_2_primitives);

    DQ p_lines_1;
    DQ p_lines_2;
    std::tie(p_lines_1,p_lines_2) = DQ_Geometry::closest_points_between_lines(line_1,line_2);

    const bool& p_lines_1_possible = is_inside_line_segment(p_lines_1,line_1_primitives);
    const bool& p_lines_2_possible = is_inside_line_segment(p_lines_2,line_2_primitives);

    double D_line_to_line = std::nan("");
    if(p_lines_1_possible && p_lines_2_possible)
    {
        D_line_to_line = DQ_Geometry::line_to_line_squared_distance(line_1,line_2);
    }

    ClosestElementsAndDistance current_result = {{Element::Line,Element::Line},
                                                 D_line_to_line
                                                };

    std::vector<Element> all_closest_elements = {Element::Line,
                                                 Element::P1,
                                                 Element::P2};


    for(const auto& closest_element_1: all_closest_elements)
    {
        for(const auto& closest_element_2: all_closest_elements)
        {
            double distance;

            switch(closest_element_1)
            {
            case Element::Line:
            {
                switch(closest_element_2)
                {
                case Element::Line:
                {
                    continue;
                }
                case Element::P1:
                {
                    distance = _line_to_point_feasibility_and_distance(line_1_primitives,
                                                                       std::get<1>(line_2_primitives));
                    break;
                }
                case Element::P2:
                {
                    distance = _line_to_point_feasibility_and_distance(line_1_primitives,
                                                                       std::get<2>(line_2_primitives));
                    break;
                }
                }
                break;
            }
            case Element::P1:
            {
                switch(closest_element_2)
                {
                case Element::Line:
                {
                    distance = _line_to_point_feasibility_and_distance(line_2_primitives,
                                                                       std::get<1>(line_1_primitives));
                    break;
                }
                case Element::P1:
                {
                    distance = DQ_Geometry::point_to_point_squared_distance(
                                std::get<1>(line_1_primitives),
                                std::get<1>(line_2_primitives)
                                );
                    break;
                }
                case Element::P2:
                {
                    distance = DQ_Geometry::point_to_point_squared_distance(
                                std::get<1>(line_1_primitives),
                                std::get<2>(line_2_primitives)
                                );
                    break;
                }
                }
                break;
            }
            case Element::P2:
            {
                switch(closest_element_2)
                {
                case Element::Line:
                {
                    distance =  _line_to_point_feasibility_and_distance(line_2_primitives,
                                                                        std::get<2>(line_1_primitives));
                    break;
                }
                case Element::P1:
                {
                    distance = DQ_Geometry::point_to_point_squared_distance(
                                std::get<2>(line_1_primitives),
                                std::get<1>(line_2_primitives)
                                );
                    break;
                }
                case Element::P2:
                {
                    distance = DQ_Geometry::point_to_point_squared_distance(
                                std::get<2>(line_1_primitives),
                                std::get<2>(line_2_primitives)
                                );
                    break;
                }
                }
                break;
            }
            }

            if(_verbose)
            {
                std::cout << "    ...Checking pair {" <<
                             to_string(closest_element_1) <<
                             "," <<
                             to_string(closest_element_2) <<
                             "}" <<
                             std::endl;
            }
            current_result = _update_closest_pair(current_result,
                                                  {{closest_element_1,closest_element_2},distance});

        }
    }

    if(_verbose)
    {
        std::cout << "Closest pair is {" <<
                     to_string(std::get<0>(std::get<0>(current_result))) <<
                     "," <<
                     to_string(std::get<1>(std::get<0>(current_result))) <<
                     "}" <<
                     std::endl;
    }

    return current_result;
}

}
}
