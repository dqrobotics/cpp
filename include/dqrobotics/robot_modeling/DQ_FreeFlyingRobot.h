/**
(C) Copyright 2023 DQ Robotics Developers

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

    1. Bruno Vilhena Adorno (adorno@ieee.org)
       Responsible for the original implementation in file DQ_FreeFlyingRobot.m
       https://github.com/dqrobotics/matlab/pull/66/commits

    2. Juan Jose Quiroz Omana   (juanjqo@g.ecc.u-tokyo.ac.jp)
       Created this file.
*/

#pragma once
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>

namespace DQ_robotics
{

class DQ_FreeFlyingRobot: public DQ_Kinematics
{
protected:

    void _invalid_signature(const std::string& msg) const;

public:
    DQ_FreeFlyingRobot();

    //Virtual method overloads (DQ_Kinematics)
    virtual DQ fkm                (const VectorXd&) const override;
    virtual DQ fkm                (const VectorXd&, const int&) const override;
    virtual MatrixXd pose_jacobian(const VectorXd&, const int&) const override;
    virtual MatrixXd pose_jacobian_derivative(const VectorXd& q,
                                              const VectorXd& q_dot,
                                              const int& to_ith_link) const override;




    DQ fkm(const DQ& pose) const;
    MatrixXd pose_jacobian(const DQ& pose) const;
    MatrixXd pose_jacobian_derivative(const DQ& pose_derivative) const;

};

}

