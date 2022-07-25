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
    along with DQ Robotics.  If !, see <http://www.gnu.org/licenses/>.

Contributors:
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
*/

#include<dqrobotics/utils/DQ_Geometry.h>
#include<dqrobotics/utils/DQ_Constants.h>

namespace DQ_robotics
{

/**
 * @brief point_to_point_square_distance obtains the squared distance between
 * @p point1 and @p point2 which have to be pure quaternions.
 * @param point1.
 * @param point2.
 * @return The squared distance between @p point1 and @p point2 as a double.
 * @exception Throws a std::range_error if either of the inputs are !
 * pure quaternions @see DQ_robotics::is_pure_quaternion().
 */
double DQ_Geometry::point_to_point_squared_distance(const DQ& point1, const DQ& point2)
{
    if(! is_pure_quaternion(point1))
    {
        throw std::range_error("Input point1 is ! a pure quaternion.");
    }
    if(! is_pure_quaternion(point2))
    {
        throw std::range_error("Input point2 is ! a pure quaternion.");
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
 * @exception Throws a std::range_error if the @p point is ! a pure quaternion
 * @see DQ_robotics::is_pure_quaternion() or if @p line is ! a Plucker line
 * @see DQ_robotics::is_line().
 */
double DQ_Geometry::point_to_line_squared_distance(const DQ& point, const DQ& line)
{
    if(! is_pure_quaternion(point))
    {
        throw std::range_error("Input point is ! a pure quaternion.");
    }
    if(! is_line(line))
    {
        throw std::range_error("Input line is ! a line.");
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
 * @exception Throws a std::range_error if the @p point is ! a pure quaternion
 * @see DQ_robotics::is_pure_quaternion() or if @p plane is ! a plane
 * @see DQ_robotics::is_plane().
 */
double DQ_Geometry::point_to_plane_distance(const DQ& point, const DQ& plane)
{
    if(! is_pure_quaternion(point))
    {
        throw std::range_error("Input point is ! a pure quaternion.");
    }
    if(! is_plane(plane))
    {
        throw std::range_error("Input plane is ! a plane.");
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
 * @p line1 or @p line2, are ! Plucker lines (@see DQ_robotics::is_line()).
 */
double DQ_Geometry::line_to_line_squared_distance(const DQ& line1, const DQ& line2)
{
    if(! is_line(line1))
    {
        throw std::range_error("Input line1 is ! a line.");
    }
    if(! is_line(line2))
    {
        throw std::range_error("Input line2 is ! a line.");
    }

    const DQ l1_cross_l2 = cross(line1,line2);
    const DQ l1_dot_l2   = dot(line1,line2);

    const double a = vec4(P(l1_cross_l2)).norm();
    const double b = vec4(D(l1_dot_l2)).norm();

    /// TODO, check input to acos to be sure it won't be negative.
    /// that happens sometimes in CPP due to rounding errors.
    const double phi = acos(static_cast<double>(P(l1_dot_l2)));

    /// TODO, add a threshold because this will never be zero.
    if( fmod(phi,pi) != 0.0)
    {
        return std::pow(static_cast<double>(b/a),2);
    }
    else
    {
        const Vector4d a = vec4(D(l1_cross_l2));
        return a.transpose()*a;
    }
}


double DQ_Geometry::line_to_line_angle(const DQ &line1, const DQ &line2)
{
    if(! is_line(line1))
    {
        throw std::range_error("Input line1 is ! a line.");
    }
    if(! is_line(line2))
    {
        throw std::range_error("Input line2 is ! a line.");
    }

    const DQ l1_dot_l2   = dot(line1,line2);

    /// TODO, check input to acos to be sure it won't be negative.
    /// that happens sometimes in CPP due to rounding errors.
    const double phi     = acos(static_cast<double>(P(l1_dot_l2)));

    return phi;
}

/**
 * @brief DQ_Geometry::point_projected_in_line
 * Obtains the point resulting from the projection of a point into a line.
 * Ref: https://faculty.sites.iastate.edu/jia/files/inline-files/plucker-coordinates.pdf
 * First used in:
 * M. M. Marinho, B. V. Adorno, K. Harada and M. Mitsuishi, "Dynamic Active Constraints for
 * Surgical Robots Using Vector-Field Inequalities," in IEEE Transactions on Robotics, vol. 35, no. 5,
 * pp. 1166-1185, Oct. 2019, doi: 10.1109/TRO.2019.2920078.
 * @param point
 * @param line
 * @return the projection of point into line as a pure quaternion.
 */
DQ DQ_Geometry::point_projected_in_line(const DQ &point, const DQ &line)
{
    const DQ& l = P(line);
    const DQ& m = D(line);


    return (l*dot(l,point)+cross(l,m));
}

/**
 * @brief DQ_Geometry::closest_points_between_lines
 * Obtains the closest points between two lines.
 * Ref: https://faculty.sites.iastate.edu/jia/files/inline-files/plucker-coordinates.pdf
 * First used in:
 * M. M. Marinho, B. V. Adorno, K. Harada and M. Mitsuishi, "Dynamic Active Constraints for
 * Surgical Robots Using Vector-Field Inequalities," in IEEE Transactions on Robotics, vol. 35, no. 5,
 * pp. 1166-1185, Oct. 2019, doi: 10.1109/TRO.2019.2920078.
 * @param line1 the first line, as a dual quaternion.
 * @param line2 the second line, as a dual quaternion.
 * @return a tuple<DQ,DQ> containing the closest point in line1 and line2, respectively.
 */
std::tuple<DQ, DQ> DQ_Geometry::closest_points_between_lines(const DQ &line1, const DQ &line2)
{
    if(!is_line(line1))
        throw std::runtime_error("DQ_Geometry::closest_points_between_lines::Input line_1 must be a line.");
    if(!is_line(line2))
        throw std::runtime_error("DQ_Geometry::closest_points_between_lines::Input line_2 must be a line.");

    const DQ& m1 = D(line1);
    const DQ& l1 = P(line1);

    const DQ& m2 = D(line2);
    const DQ& l2 = P(line2);

    const DQ& p1 = (cross(-1.0*m1,cross(l2,cross(l1,l2))) + dot(m2,cross(l1,l2) )*l1 )*std::pow(1./vec4(cross(l1,l2)).norm(),2);
    const DQ& p2 = (cross( m2,cross(l1,cross(l1,l2))) - dot(m1,cross(l1,l2) )*l2 )*std::pow(1./vec4(cross(l1,l2)).norm(),2);

    return {p1,p2};
}

/**
 * @brief DQ_Geometry::is_line_segment
 * Check if line, line_point_1, and line_point_2 constitute a line segment.
 * Verifies if:
 * 1. Line is a line
 * 2. line_point_1 and line_point_2 are points.
 * 3. line_point_1 and line_point_2 are contained in line. This last check can be too sensitive
 * when using DQ_Threshold, for instance if the arguments come from another library with less precision.
 * In that case, the threshold optional parameter can be used.
 * @param line the Plucker line to be checked.
 * @param line_point_1 one end of the line segment, must be on line.
 * @param line_point_2 the other end of the line segment, must be on line.
 * @param threshold a threshold for the check for the points to be on the line. Default DQ_threshold.
 * @return true if a line segment, false otherwise.
 */
bool DQ_Geometry::is_line_segment(const DQ &line,
                                  const DQ &line_point_1,
                                  const DQ &line_point_2,
                                  const double& threshold)
{
    if(!is_line(line))
        return false;
    if(!is_pure_quaternion(line_point_1))
        return false;
    if(!is_pure_quaternion(line_point_2))
        return false;

    const DQ& m = D(line);
    const DQ& l = P(line);

    if((cross(line_point_1,l) - m).vec3().norm() > threshold)
        return false;
    if((cross(line_point_2,l) - m).vec3().norm() > threshold)
        return false;

    return true;
}

/**
 * @brief DQ_Geometry::closest_points_between_line_segments
 * Obtains the closest point between two line segments.
 * @see DQ_Geometry::closest_points_between_lines
 * First used in:
 * M. M. Marinho, B. V. Adorno, K. Harada and M. Mitsuishi, "Dynamic Active Constraints for
 * Surgical Robots Using Vector-Field Inequalities," in IEEE Transactions on Robotics, vol. 35, no. 5,
 * pp. 1166-1185, Oct. 2019, doi: 10.1109/TRO.2019.2920078.
 * @param line_1 the first line.
 * @param line_1_point_1 the first point delimiting the first line. Must be on line_1.
 * @param line_1_point_2 the second point delimiting the first line. Must be on line_1.
 * @param line_2 the second line.
 * @param line_2_point_1 the first point delimiting the second line. Must be on line_2.
 * @param line_2_point_2 the second point delimiting the second line. Must be on line_2.
 * @return a tuple<DQ,DQ> containing the closest point between the two line segments.
 */
std::tuple<DQ, DQ> DQ_Geometry::closest_points_between_line_segments(const DQ &line_1,
                                                                     const DQ &line_1_point_1,
                                                                     const DQ &line_1_point_2,
                                                                     const DQ &line_2,
                                                                     const DQ &line_2_point_1,
                                                                     const DQ &line_2_point_2)
{
    if(!is_line_segment(line_1,line_1_point_1,line_1_point_2))
        throw std::runtime_error("DQ_Geometry::closest_points_between_line_segments::Input line_1, line_1_point_1, "
                                 "and line_1_point_2 must contitute a valid line segment.");
    if(!is_line_segment(line_2,line_2_point_1,line_2_point_2))
        throw std::runtime_error("DQ_Geometry::closest_points_between_line_segments::Input line_2, line_2_point_1, "
                                 "and line_2_point_2 must contitute a valid line segment");
    const DQ& l1 = P(line_1);
    const DQ& l2 = P(line_2);

    if( l1 == l2 || l1 == -l2) //TODO, add a tolerance interval
    {
        //In this case, it is impossible to find a single closest point because all points
        std::range_error("DQ_Geometry::closest_points_between_line_segments::Lines are parallel, impossible to find a unique closest point.");
    }
    else
    {
        //In this case, the closest points (cps) can be found
        DQ cp1;
        DQ cp2;
        std::tie(cp1,cp2) = DQ_Geometry::closest_points_between_lines(line_1,line_2);

        ///Find out the closest pairs
        //Get the distance between all relevant points
        const double& segment_1_size = DQ_Geometry::point_to_point_squared_distance(line_1_point_1,line_1_point_2);
        const double& D_cp1_l1p1 = DQ_Geometry::point_to_point_squared_distance(cp1,line_1_point_1);
        const double& D_cp1_l1p2 = DQ_Geometry::point_to_point_squared_distance(cp1,line_1_point_2);
        const double& segment_2_size = DQ_Geometry::point_to_point_squared_distance(line_2_point_1,line_2_point_2);
        const double& D_cp2_l2p1 = DQ_Geometry::point_to_point_squared_distance(cp2,line_2_point_1);
        const double& D_cp2_l2p2 = DQ_Geometry::point_to_point_squared_distance(cp2,line_2_point_2);

        //Closest element (ce) local enum class
        enum class ClosestElement{
            LINE,P1,P2
        };
        ClosestElement ce1;
        ClosestElement ce2;

        if(D_cp1_l1p1 < segment_1_size && D_cp1_l1p2 < segment_1_size)
            ce1 = ClosestElement::LINE;
        else if( D_cp1_l1p1 < D_cp1_l1p2)
            ce1 = ClosestElement::P1;
        else
            ce1 = ClosestElement::P2;

        if(D_cp2_l2p1 < segment_2_size && D_cp2_l2p2 < segment_2_size)
            ce2 = ClosestElement::LINE;
        else if( D_cp2_l2p1 < D_cp2_l2p2)
            ce2 = ClosestElement::P1;
        else
            ce2 = ClosestElement::P2;


        switch(ce1)
        {
        case ClosestElement::LINE:
        {
            switch(ce2)
            {
            case ClosestElement::LINE:
                return DQ_Geometry::closest_points_between_lines(line_1,line_2);
            case ClosestElement::P1:
                return {DQ_Geometry::point_projected_in_line(line_2_point_1,line_1),
                            line_2_point_1};
            case ClosestElement::P2:
                return {DQ_Geometry::point_projected_in_line(line_2_point_2,line_1),
                            line_2_point_2};
            }
            throw std::runtime_error("Unexpected type in DQ_Geometry::closest_points_between_line_segments()");
        }
        case ClosestElement::P1:
        {
            switch(ce2)
            {
            case ClosestElement::LINE:
                return {line_1_point_1,
                        DQ_Geometry::point_projected_in_line(line_1_point_1,line_2)};
            case ClosestElement::P1:
                return {line_1_point_1,line_2_point_1};
            case ClosestElement::P2:
                return {line_1_point_1, line_2_point_2};
            }
            throw std::runtime_error("Unexpected type in DQ_Geometry::closest_points_between_line_segments()");
        }
        case ClosestElement::P2:
        {
            switch(ce2)
            {
            case ClosestElement::LINE:
                return {line_1_point_2,
                            DQ_Geometry::point_projected_in_line(line_1_point_2,line_2)};
            case ClosestElement::P1:
                return {line_1_point_2,line_2_point_1};
            case ClosestElement::P2:
                return {line_1_point_2, line_2_point_2};
            }
            throw std::runtime_error("Unexpected type in DQ_Geometry::closest_points_between_line_segments()");
        }
        default:
            throw std::runtime_error("Unexpected type in DQ_Geometry::closest_points_between_line_segments()");
        }

    }
    throw std::runtime_error("Unexpected end of method in DQ_Geometry::closest_points_between_line_segments()");
}

/**
 * @brief DQ_Geometry::line_segment_to_line_squared_distance
 * Obtains the squared distance between two line segments.
 * This method will check the validity of all dual quaternion inputs, but has no way to check
 * the validity of the Jacobians.
 *
 * TODO: Murilo: This initial implementation replicates a lot of what is done in
 * DQ_Kinematics::line_segment_to_line_segment_distance_jacobian
 * and most likely a better approach is possible.
 *
 * As mentioned in Section VI of
 * M. M. Marinho, B. V. Adorno, K. Harada and M. Mitsuishi, "Dynamic Active Constraints for
 * Surgical Robots Using Vector-Field Inequalities," in IEEE Transactions on Robotics, vol. 35, no. 5,
 * pp. 1166-1185, Oct. 2019, doi: 10.1109/TRO.2019.2920078.
 * @param line_1 the first line.
 * @param line_1_point_1 the first point delimiting the first line. Must be on line_1.
 * @param line_1_point_2 the second point delimiting the first line. Must be on line_1.
 * @param line_2 the second line.
 * @param line_2_point_1 the first point delimiting the second line. Must be on line_2.
 * @param line_2_point_2 the second point delimiting the second line. Must be on line_2.
 * @return the squared distance between line_1 and line_2
 */
double DQ_Geometry::line_segment_to_line_segment_squared_distance(const DQ& line_1,
                                                                  const DQ& line_1_point_1,
                                                                  const DQ& line_1_point_2,
                                                                  const DQ& line_2,
                                                                  const DQ& line_2_point_1,
                                                                  const DQ& line_2_point_2)
{
    if(!is_line_segment(line_1,line_1_point_1,line_1_point_2))
        throw std::runtime_error("DQ_Geometry::line_segment_to_line_squared_distance::Input line_1, line_1_point_1, "
                                 "and line_1_point_2 must contitute a valid line segment.");
    if(!is_line_segment(line_2,line_2_point_1,line_2_point_2))
        throw std::runtime_error("DQ_Geometry::line_segment_to_line_squared_distance::Input line_2, line_2_point_1, "
                                 "and line_2_point_2 must contitute a valid line segment");

    const DQ& l1 = P(line_1);
    const DQ& l2 = P(line_2);

    if( l1 == l2 || l1 == -l2) //TODO, add a tolerance interval
    {
        //In this case, it is impossible to find a single closest point because all points
        //have equal distance between lines
        return DQ_Geometry::line_to_line_squared_distance(line_1, line_2);
    }
    else
    {
        //In this case, the closest points (cps) can be found
        DQ cp1;
        DQ cp2;
        std::tie(cp1,cp2) = DQ_Geometry::closest_points_between_lines(line_1,line_2);

        ///Find out the closest pairs
        //Get the distance between all relevant points
        const double& segment_1_size = DQ_Geometry::point_to_point_squared_distance(line_1_point_1,line_1_point_2);
        const double& D_cp1_l1p1 = DQ_Geometry::point_to_point_squared_distance(cp1,line_1_point_1);
        const double& D_cp1_l1p2 = DQ_Geometry::point_to_point_squared_distance(cp1,line_1_point_2);
        const double& segment_2_size = DQ_Geometry::point_to_point_squared_distance(line_2_point_1,line_2_point_2);
        const double& D_cp2_l2p1 = DQ_Geometry::point_to_point_squared_distance(cp2,line_2_point_1);
        const double& D_cp2_l2p2 = DQ_Geometry::point_to_point_squared_distance(cp2,line_2_point_2);

        //Closest element (ce) local enum class
        enum class ClosestElement{
            LINE,P1,P2
        };
        ClosestElement ce1;
        ClosestElement ce2;

        if(D_cp1_l1p1 < segment_1_size && D_cp1_l1p2 < segment_1_size)
            ce1 = ClosestElement::LINE;
        else if( D_cp1_l1p1 < D_cp1_l1p2)
            ce1 = ClosestElement::P1;
        else
            ce1 = ClosestElement::P2;

        if(D_cp2_l2p1 < segment_2_size && D_cp2_l2p2 < segment_2_size)
            ce2 = ClosestElement::LINE;
        else if( D_cp2_l2p1 < D_cp2_l2p2)
            ce2 = ClosestElement::P1;
        else
            ce2 = ClosestElement::P2;


        switch(ce1)
        {
        case ClosestElement::LINE:
        {
            switch(ce2)
            {
            case ClosestElement::LINE:
                return DQ_Geometry::line_to_line_squared_distance(line_1,line_2);
            case ClosestElement::P1:
                return DQ_Geometry::point_to_line_squared_distance(line_2_point_1,line_1);
            case ClosestElement::P2:
                return DQ_Geometry::point_to_line_squared_distance(line_2_point_2,line_1);
            }
            throw std::runtime_error("Unexpected type in DQ_Geometry::line_segment_to_line_squared_distance()");
        }
        case ClosestElement::P1:
        {
            switch(ce2)
            {
            case ClosestElement::LINE:
                return DQ_Geometry::point_to_line_squared_distance(line_1_point_1,line_2);
            case ClosestElement::P1:
                return DQ_Geometry::point_to_point_squared_distance(line_1_point_1,line_2_point_1);
            case ClosestElement::P2:
                return DQ_Geometry::point_to_point_squared_distance(line_1_point_1,line_2_point_2);
            }
            throw std::runtime_error("Unexpected type in DQ_Geometry::line_segment_to_line_squared_distance()");
        }
        case ClosestElement::P2:
        {
            switch(ce2)
            {
            case ClosestElement::LINE:
                return DQ_Geometry::point_to_line_squared_distance(line_1_point_2,line_2);
            case ClosestElement::P1:
                return DQ_Geometry::point_to_point_squared_distance(line_1_point_2,line_2_point_1);
            case ClosestElement::P2:
                return DQ_Geometry::point_to_point_squared_distance(line_1_point_2,line_2_point_2);
            }
            throw std::runtime_error("Unexpected type in DQ_Geometry::line_segment_to_line_squared_distance()");
        }
        default:
            throw std::runtime_error("Unexpected type in DQ_Geometry::line_segment_to_line_squared_distance()");
        }

    }
    throw std::runtime_error("Unexpected end of method in DQ_Geometry::line_segment_to_line_squared_distance()");
}


}
