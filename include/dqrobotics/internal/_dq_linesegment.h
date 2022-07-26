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
- Murilo M. Marinho (murilo@g.ecc.u-tokyo.ac.jp)
*/
#include <dqrobotics/DQ.h>
#include <tuple>

namespace DQ_robotics
{


namespace internal
{

class LineSegment
{
public:
    enum class Element{
        Line,P1,P2
    };

    using Primitives = std::tuple<DQ,DQ,DQ>;
    using ClosestElements = std::tuple<Element,Element>;
    using ClosestElementsAndDistance = std::tuple<ClosestElements, double>;

    static ClosestElementsAndDistance closest_elements_between_line_segments(const Primitives& line_1_primitives,
                                                                             const Primitives& line_2_primitives);


    static bool is_inside_line_segment(const DQ &point, const Primitives &line_1_primitives);

    static std::string to_string(const Element& e);


private:

    static ClosestElementsAndDistance _update_closest_pair(
            const ClosestElementsAndDistance& current,
            const ClosestElementsAndDistance& candidate);

    static double _line_to_point_feasibility_and_distance(
            const Primitives& line_segment,
            const DQ& point);
};

}

}
