/**
(C) Copyright 2021 DQ Robotics Developers

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
- Murilo M. Marinho (murilo@g.ecc.u-tokyo.ac.jp)
*/
#pragma once

#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>

namespace DQ_robotics
{

class DQ_SerialManipulatorDenso: public DQ_SerialManipulator
{
protected:
    MatrixXd    denso_matrix_;

    DQ _denso2dh(const double& q, const int& ith) const;
public:

    DQ_SerialManipulatorDenso()=delete;
    DQ_SerialManipulatorDenso(const MatrixXd& denso_matrix);

    VectorXd get_as() const;
    VectorXd get_bs() const;
    VectorXd get_ds() const;
    VectorXd get_alphas() const;
    VectorXd get_betas() const;
    VectorXd get_gammas() const;

    //Using
    using DQ_SerialManipulator::raw_pose_jacobian;
    using DQ_SerialManipulator::raw_fkm;

    //Override from DQ_SerialManipulator
    MatrixXd raw_pose_jacobian(const VectorXd& q_vec, const int& to_ith_link) const override;
    DQ raw_fkm(const VectorXd &q_vec, const int &to_ith_link) const override;
};

}//Namespace DQRobotics
