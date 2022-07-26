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
- Murilo M. Marinho (murilo@g.ecc.u-tokyo.ac.jp)
*/
#include <dqrobotics/internal/_dq_linesegment.h>
#include <dqrobotics/utils/DQ_Geometry.h>
#include <vector>

namespace DQ_robotics
{
namespace internal
{

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

double LineSegment::get_squared_distance_for_primitive_pair(const Element &element_1_type,
                                                            const DQ &element_1,
                                                            const Element &element_2_type,
                                                            const DQ &element_2)
{
    switch(element_1_type)
    {
    case Element::Line:
    {
        switch(element_2_type)
        {
        case Element::Line:
            return DQ_Geometry::line_to_line_squared_distance(element_1,element_2);
        case Element::P1:
            //fallthrough
        case Element::P2:
            return DQ_Geometry::point_to_line_squared_distance(element_2,element_1);
        }
    }
    case Element::P1:
        //fallthrough
    case Element::P2:
    {
        switch(element_2_type)
        {
        case Element::Line:
            return DQ_Geometry::point_to_line_squared_distance(element_1,element_2);
        case Element::P1:
            //fallthrough
        case Element::P2:
            return DQ_Geometry::point_to_line_squared_distance(element_1,element_2);
        }
    }
    }
    throw std::runtime_error("Unexpected end of function");
}

LineSegment::ClosestElementsAndDistance LineSegment::_update_closest_pair(const ClosestElementsAndDistance &current, const ClosestElementsAndDistance &candidate)
{
    if(std::get<1>(current)==std::nan(""))
    {
        if(std::get<1>(candidate)==std::nan(""))
            return current;// Both are invalid so it doesn't really matter
        else
            return candidate;
    }
    else
    {
        if(std::get<1>(candidate) < std::get<1>(current))
            return candidate;
        else
            return current;
    }
}

double LineSegment::_line_to_point_feasibility_and_distance(const Primitives &line_segment, const DQ &point)
{
    bool is_possible = is_inside_line_segment(point, line_segment);
    if(is_possible)
    {
        return DQ_Geometry::point_to_line_squared_distance(point,
                                                           std::get<0>(line_segment)
                                                           );
    }
    return std::nan("");
}

LineSegment::ClosestElementsAndDistance
LineSegment::closest_elements_between_line_segments(const Primitives& line_1_primitives,
                                                    const Primitives& line_2_primitives)
{
    /// Case 1 : Line to Line
    const auto& line_1 = std::get<0>(line_1_primitives);
    const auto& line_2 = std::get<0>(line_2_primitives);

    DQ p_lines_1;
    DQ p_lines_2;
    std::tie(p_lines_1,p_lines_2) = DQ_Geometry::closest_points_between_lines(line_1,line_2);

    bool p_lines_1_possible = is_inside_line_segment(p_lines_1,line_1_primitives);
    bool p_lines_2_possible = is_inside_line_segment(p_lines_2,line_2_primitives);

    double D_line_to_line = std::nan("");
    bool line_line_distance_possible = p_lines_1_possible && p_lines_2_possible;
    if(line_line_distance_possible)
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
                    distance = _line_to_point_feasibility_and_distance(line_1_primitives,std::get<1>(line_2_primitives));
                    break;
                }
                case Element::P2:
                {
                    distance = _line_to_point_feasibility_and_distance(line_1_primitives,std::get<2>(line_2_primitives));
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
                    continue;
                }
                case Element::P1:
                {
                    distance = _line_to_point_feasibility_and_distance(line_1_primitives,std::get<1>(line_2_primitives));
                    break;
                }
                case Element::P2:
                {
                    distance = _line_to_point_feasibility_and_distance(line_1_primitives,std::get<2>(line_2_primitives));
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
                    distance =  _line_to_point_feasibility_and_distance(line_2_primitives,std::get<2>(line_1_primitives));
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

            current_result = _update_closest_pair(current_result,
                                                  {{closest_element_1,closest_element_2},distance});
        }
    }
    return current_result;
}

}
}
