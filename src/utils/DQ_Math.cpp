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
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
*/
#include <dqrobotics/utils/DQ_Math.h>

namespace DQ_robotics
{


/**
 * @brief deg2rad
 * @param v
 * @return
 */
VectorXd deg2rad(const VectorXd& v)
{
    VectorXd ret(v);
    for(auto i=0;i<v.size();i++)
    {
        ret(i) = deg2rad(v(i));
    }
    return ret;
}

/**
 * @brief rad2deg
 * @param v
 * @return
 */
VectorXd rad2deg(const VectorXd& v)
{
    VectorXd ret(v);
    for(auto i=0;i<v.size();i++)
    {
        ret(i) = rad2deg(v(i));
    }
    return ret;
}

}
