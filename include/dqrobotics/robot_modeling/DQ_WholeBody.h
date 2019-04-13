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

#include<vector>
#include<dqrobotics/DQ.h>
#include<dqrobotics/robot_modeling/DQ_Kinematics.h>

namespace DQ_robotics
{

class DQ_WholeBody : DQ_Kinematics
{
protected:
    std::vector<DQ_Kinematics*> chain_;
    int dim_configuration_space_;
public:
    DQ_WholeBody(DQ_Kinematics* robot);

    void add(const DQ_Kinematics& robot);
    DQ fkm(const VectorXd& q, const int& to_link) const;

    //Abstract methods' implementation
    int get_dim_configuration_space() const;
    DQ fkm(const VectorXd& q) const;
    MatrixXd pose_jacobian(const VectorXd& q, const int& to_link) const;

};

}
