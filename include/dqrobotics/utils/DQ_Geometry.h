#pragma once
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
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
*/


#include<dqrobotics/DQ.h>
#include<tuple>

namespace DQ_robotics
{

class DQ_Geometry
{
public:
    static double point_to_point_squared_distance(const DQ& point1, const DQ& point2);

    static double point_to_line_squared_distance(const DQ& point, const DQ& line);

    static double point_to_plane_distance(const DQ& point, const DQ& plane);

    static double line_to_line_squared_distance(const DQ& line1, const DQ& line2);

    static double line_to_line_angle(const DQ& line1, const DQ& line2);

    static DQ point_projected_in_line(const DQ& point, const DQ& line);

    static std::tuple<DQ,DQ> closest_points_between_lines(const DQ& line1, const DQ& line2);

    static bool is_line_segment(const DQ& line,
                                const DQ& line_point_1,
                                const DQ& line_point_2,
                                const double& threshold=DQ_threshold);

    static std::tuple<DQ,DQ> closest_points_between_line_segments(const DQ& line_1,
                                                                  const DQ& line_1_point_1,
                                                                  const DQ& line_1_point_2,
                                                                  const DQ& line_2,
                                                                  const DQ& line_2_point_1,
                                                                  const DQ& line_2_point_2);

    static double line_segment_to_line_segment_squared_distance(const DQ& line_1,
                                                                const DQ& line_1_point_1,
                                                                const DQ& line_1_point_2,
                                                                const DQ& line_2,
                                                                const DQ& line_2_point_1,
                                                                const DQ& line_2_point_2);

};

}


