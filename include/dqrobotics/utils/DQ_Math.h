#pragma once
/**
(C) Copyright 2020-2022 DQ Robotics Developers

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
#include<dqrobotics/utils/DQ_Constants.h>

namespace DQ_robotics
{

/**
 * @brief Converts an angle from degrees to radians.
 *
 * @param a The angle in degrees.
 * @return The same angle expressed in radians.
 * @see rad2deg
 */
constexpr double deg2rad(const double& a) noexcept
{
    return (a)*pi/(180.0);
}

/**
 * @brief Converts each component of a vector from degrees to radians.
 *
 * @param v The input vector in degrees.
 * @return A vector whose entries are the corresponding values in radians.
 * @see rad2deg
 */
VectorXd deg2rad(const VectorXd& v);

/**
 * @brief Converts an angle from radians to degrees.
 *
 * @param a The angle in radians.
 * @return The same angle expressed in degrees.
 * @see deg2rad
 */
constexpr double rad2deg(const double& a) noexcept
{
    return (a)*180.0/(pi);
}

/**
 * @brief Converts each component of a vector from radians to degrees.
 *
 * @param v The input vector in radians.
 * @return A vector whose entries are the corresponding values in degrees.
 * @see deg2rad
 */
VectorXd rad2deg(const VectorXd& v);

}
