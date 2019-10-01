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

#ifndef DQ_ROBOT_MODELLING_DQ_WHOLE_BODY_H
#define DQ_ROBOT_MODELLING_DQ_WHOLE_BODY_H

#include<vector>
#include<memory>
#include<dqrobotics/DQ.h>
#include<dqrobotics/robot_modeling/DQ_Kinematics.h>
#include<dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include<dqrobotics/robot_modeling/DQ_HolonomicBase.h>

namespace DQ_robotics
{

class DQ_WholeBody : public DQ_Kinematics
{
protected:
    //The most common use-case I can imagine is for the DQ_Kinematics
    //instanced to generate the caller thread to go out of scope
    //fairly soon.
    //So I'm using a shared_ptr to extend the lifetime of the instance.
    //If a raw pointer is used instead we will have a segfault fest.
    std::vector<std::shared_ptr<DQ_Kinematics>> chain_;
    int dim_configuration_space_;
public:
    DQ_WholeBody()=delete;
    DQ_WholeBody(std::shared_ptr<DQ_Kinematics> robot);

    void add(std::shared_ptr<DQ_Kinematics> robot);
    //Starts count from 0
    DQ raw_fkm(const VectorXd& q, const int& to_chain) const;
    DQ raw_fkm(const VectorXd& q) const;
    void set_effector(const DQ& effector);
    DQ_Kinematics* get_chain(const int& index);
    DQ_SerialManipulator* get_chain_as_serial_manipulator(const int& index);
    DQ_HolonomicBase* get_chain_as_holonomic_base(const int& index);

    //Abstract methods' implementation
    int get_dim_configuration_space() const override;
    DQ fkm(const VectorXd& q) const override;
    DQ fkm(const VectorXd&, const int& to_chain) const;
    MatrixXd pose_jacobian(const VectorXd& q, const int& to_link) const override;
};

}

#endif
