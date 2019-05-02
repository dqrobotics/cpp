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

#include<dqrobotics/utils/DQ_Geometry.h>

namespace DQ_robotics
{

/**
 * @brief point_to_point_square_distance obtains the squared distance between
 * @p point1 and @p point2 which have to be pure quaternions.
 * @param point1.
 * @param point2.
 * @return The squared distance between @p point1 and @p point2 as a double.
 * @exception Throws a std::range_error if either of the inputs are not
 * pure quaternions @see DQ_robotics::is_pure_quaternion().
 */
double DQ_Geometry::point_to_point_squared_distance(const DQ& point1, const DQ& point2)
{
    if(not is_pure_quaternion(point1))
    {
        throw std::range_error("Input point1 is not a pure quaternion.");
    }
    if(not is_pure_quaternion(point2))
    {
        throw std::range_error("Input point2 is not a pure quaternion.");
    }

    const Vector4d a = vec4(point1-point2);
    return a.transpose()*a;
}

/**
 * @brief point_to_line_squared_distance obtains the squared distance between @p point
 * and @p line.
 * @param point.
 * @param line.
 * @return The squared distance between @p point and @p line as a double.
 * @exception Throws a std::range_error if the @p point is not a pure quaternion
 * @see DQ_robotics::is_pure_quaternion() or if @p line is not a Plucker line
 * @see DQ_robotics::is_line().
 */
double DQ_Geometry::point_to_line_squared_distance(const DQ& point, const DQ& line)
{
    if(not is_pure_quaternion(point))
    {
        throw std::range_error("Input point is not a pure quaternion.");
    }
    if(not is_line(line))
    {
        throw std::range_error("Input line is not a line.");
    }

    const DQ l = P(line);
    const DQ m = D(line);

    const Vector4d a = vec4(cross(point,l)-m);
    return a.transpose()*a;
}

/**
 * @brief point_to_plane_distance obtains the distance between @p point
 * and @p plane.
 * @param point.
 * @param plane.
 * @return The distance between @p point and @p plane as a double.
 * @exception Throws a std::range_error if the @p point is not a pure quaternion
 * @see DQ_robotics::is_pure_quaternion() or if @p plane is not a plane
 * @see DQ_robotics::is_plane().
 */
double DQ_Geometry::point_to_plane_distance(const DQ& point, const DQ& plane)
{
    if(not is_pure_quaternion(point))
    {
        throw std::range_error("Input point is not a pure quaternion.");
    }
    if(not is_plane(plane))
    {
        throw std::range_error("Input plane is not a plane.");
    }

    const DQ plane_n = P(plane);
    const DQ plane_d = D(plane);

    return static_cast<double>((dot(point,plane_n)-plane_d));
}

/**
 * @brief line_to_line_squared_distance obtains the squared distance between @p line1
 * and @p line2.
 * @param line1.
 * @param line2.
 * @return The squared distance between @p line1 and @p line2 as a double.
 * @exception Throws a std::range_error if either of the input lines,
 * @p line1 or @p line2, are not Plucker lines (@see DQ_robotics::is_line()).
 */
double DQ_Geometry::line_to_line_squared_distance(const DQ& line1, const DQ& line2)
{
    if(not is_line(line1))
    {
        throw std::range_error("Input line1 is not a line.");
    }
    if(not is_line(line2))
    {
        throw std::range_error("Input line2 is not a line.");
    }

    const DQ l1_cross_l2 = cross(line1,line2);
    const DQ l1_dot_l2   = dot(line1,line2);

    const double a = vec4(P(l1_cross_l2)).norm();
    const double b = vec4(D(l1_dot_l2)).norm();

    /// TODO, check input to acos to be sure it won't be negative.
    /// that happens sometimes in CPP due to rounding errors.
    const double phi = acos(static_cast<double>(P(l1_dot_l2)));

    /// TODO, add a threshold because this will never be zero.
    if( fmod(phi,M_PI) != 0.0)
    {
        return std::pow(static_cast<double>(b/a),2);
    }
    else
    {
        const Vector4d a = vec4(D(l1_cross_l2));
        return a.transpose()*a;
    }
}

}
