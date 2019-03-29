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

#include<dqrobotics/DQ_CooperativeDualTaskSpace.h>

namespace DQ_robotics
{

DQ_CooperativeDualTaskSpace::DQ_CooperativeDualTaskSpace(const DQ_kinematics& robot1, const DQ_kinematics& robot2)
{
    robot1_ = robot1;
    robot2_ = robot2;
}

DQ DQ_CooperativeDualTaskSpace::pose1(const VectorXd &theta) const
{
    return robot1_.fkm(theta.head(robot1_.n_links()));
}

DQ DQ_CooperativeDualTaskSpace::pose2(const VectorXd &theta) const
{
    return robot2_.fkm(theta.tail(robot2_.n_links()));
}

MatrixXd DQ_CooperativeDualTaskSpace::pose_jacobian1(const VectorXd &theta) const
{
    return robot1_.pose_jacobian(theta.head(robot1_.n_links()));
}

MatrixXd DQ_CooperativeDualTaskSpace::pose_jacobian2(const VectorXd &theta) const
{
    return robot2_.pose_jacobian(theta.tail(robot2_.n_links()));
}

DQ DQ_CooperativeDualTaskSpace::relative_pose(const VectorXd &theta) const
{
    return conj(pose2(theta))*pose1(theta);
}

DQ DQ_CooperativeDualTaskSpace::absolute_pose(const VectorXd &theta) const
{
    return pose2(theta)*(pow(relative_pose(theta),0.5));
}

MatrixXd DQ_CooperativeDualTaskSpace::relative_pose_jacobian(const VectorXd &theta) const
{
    const MatrixXd Jx1 = pose_jacobian1(theta);
    const DQ       x1  = pose1(theta);
    const MatrixXd Jx2 = pose_jacobian2(theta);
    const DQ       x2  = pose2(theta);

    MatrixXd Jxr(8,Jx1.cols()+Jx2.cols());
    Jxr << (hamiplus8(conj(x2)))*Jx1,haminus8(x1)*C8()*Jx2;
    return  Jxr;
}

MatrixXd DQ_CooperativeDualTaskSpace::absolute_pose_jacobian(const VectorXd &theta) const
{
    //Preliminaries
    const MatrixXd Jx2 = pose_jacobian2(theta);
    const DQ       x2  = pose2(theta);
    const MatrixXd Jxr  = relative_pose_jacobian(theta);
    const DQ       xr  = relative_pose(theta);
    const MatrixXd Jtr = translation_jacobian(Jxr,xr);

    //Rotation part
    MatrixXd Jrr2    = 0.5*haminus4(conj(xr.P())*pow(xr.P(),0.5))*Jxr.block(0,0,4,Jxr.cols());

    MatrixXd Jxr2(8,Jrr2.cols());
    Jxr2 << Jrr2, 0.25*(haminus4(pow(xr.P(),0.5))*Jtr + hamiplus4(translation(xr))*Jrr2);

    MatrixXd temp(8,robot1_.n_links()+robot2_.n_links());
    temp << MatrixXd::Zero(8,robot1_.n_links()),Jx2;

    MatrixXd Jxa(8,robot1_.n_links()*robot2_.n_links());
    Jxa << haminus8(pow(xr,0.5))*(temp) + hamiplus8(x2)*Jxr2;

    return Jxa;
}

}
