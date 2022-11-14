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

#ifndef DQ_ROBOTICS_ROBOT_MODELING_DQ_MOBILEBASE
#define DQ_ROBOTICS_ROBOT_MODELING_DQ_MOBILEBASE

#include<dqrobotics/DQ.h>
#include<dqrobotics/robot_modeling/DQ_Kinematics.h>

namespace DQ_robotics
{

class DQ_MobileBase : public DQ_Kinematics
{
protected:
    DQ frame_displacement_;

    DQ_MobileBase();
public:
    virtual ~DQ_MobileBase() = default;

    //Abstract methods (Inherited from DQ_Kinematics)
    //virtual int      get_dim_configuration_space() const = 0;
    //virtual DQ       fkm(const VectorXd& joint_configurations) const = 0;
    //virtual MatrixXd pose_jacobian(const VectorXd& joint_configurations,const int& to_link) const = 0;
    //virtual MatrixXd pose_jacobian_derivative(const VectorXd& configurations, const VectorXd& velocity_configurations, const int& to_link) const = 0;

    void set_frame_displacement(const DQ& pose);
    DQ   frame_displacement();

};

}

#endif
