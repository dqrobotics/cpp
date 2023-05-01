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

constexpr double deg2rad(const double& a) noexcept
{
    return (a)*pi/(180.0);
}

VectorXd deg2rad(const VectorXd& v);

constexpr double rad2deg(const double& a) noexcept
{
    return (a)*180.0/(pi);
}

VectorXd rad2deg(const VectorXd& v);

}
