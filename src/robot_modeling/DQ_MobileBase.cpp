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

#include<dqrobotics/robot_modeling/DQ_MobileBase.h>

namespace DQ_robotics
{

DQ_MobileBase::DQ_MobileBase()
{
    frame_displacement_ = DQ(1);
}

DQ DQ_MobileBase::frame_displacement()
{
    return frame_displacement_;
}

void DQ_MobileBase::set_frame_displacement(const DQ &pose)
{
    frame_displacement_ = pose;
}

}
