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

#include<dqrobotics/robot_modelling/DQ_Kinematics.h>

namespace DQ_robotics
{

/* **********************************************************************
 *  CONSTRUCTOR
 * *********************************************************************/
DQ_Kinematics::DQ_Kinematics()
{
    reference_frame_ = DQ(1);
    base_frame_      = DQ(1);
}

/* **********************************************************************
 *  CONCRETE METHODS
 * *********************************************************************/

void DQ_Kinematics::set_reference_frame(const DQ &reference_frame)
{
    reference_frame_ = reference_frame;
}

DQ   DQ_Kinematics::reference_frame() const
{
    return reference_frame_;
}

void DQ_Kinematics::set_base_frame(const DQ &base_frame)
{
    base_frame_ = base_frame;
}

DQ DQ_Kinematics::base_frame() const
{
    return base_frame_;
}

void DQ_Kinematics::set_name(const std::string &name)
{
    name_ = name;
}

std::string DQ_Kinematics::name() const
{
    return name_;
}

/* **********************************************************************
 *  STATIC METHODS
 * *********************************************************************/

MatrixXd DQ_Kinematics::distance_jacobian(const MatrixXd &pose_jacobian, const DQ &pose)
{
    const DQ t        = translation(pose);
    const MatrixXd Jt = DQ_Kinematics::translation_jacobian(pose_jacobian,pose);
    const MatrixXd Jd = 2*vec4(t).transpose()*Jt;
    return Jd;
}

MatrixXd DQ_Kinematics::translation_jacobian(const MatrixXd &pose_jacobian, const DQ &pose)
{
    return 2.0*haminus4(conj(P(pose)))*pose_jacobian.block(4,0,8,pose_jacobian.cols())+2.0*hamiplus4(D(pose))*C4()*DQ_Kinematics::rotation_jacobian(pose_jacobian);
}


MatrixXd DQ_Kinematics::rotation_jacobian(const MatrixXd &pose_jacobian)
{
    return pose_jacobian.block(0,0,4,pose_jacobian.cols());
}

}
