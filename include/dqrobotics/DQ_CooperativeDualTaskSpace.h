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
- Murilo M. Marinho        (murilo@nml.t.u-tokyo.ac.jp)
*/

#ifndef DQ_COOPERATIVEDUALTASKSPACE_H
#define DQ_COOPERATIVEDUALTASKSPACE_H

#include<dqrobotics/DQ.h>
#include<dqrobotics/DQ_kinematics.h>

namespace DQ_robotics
{

class DQ_CooperativeDualTaskSpace
{
private:
    DQ_kinematics robot1_;
    DQ_kinematics robot2_;

public:
    DQ_CooperativeDualTaskSpace(const DQ_kinematics& robot1, const DQ_kinematics& robot2);

    DQ pose1(const VectorXd& theta) const;
    DQ pose2(const VectorXd& theta) const;

    MatrixXd pose_jacobian1(const VectorXd& theta) const;
    MatrixXd pose_jacobian2(const VectorXd& theta) const;

    DQ relative_pose(const VectorXd& theta) const;
    DQ absolute_pose(const VectorXd& theta) const;

    MatrixXd relative_pose_jacobian(const VectorXd& theta) const;
    MatrixXd absolute_pose_jacobian(const VectorXd& theta) const;

};


}

#endif
