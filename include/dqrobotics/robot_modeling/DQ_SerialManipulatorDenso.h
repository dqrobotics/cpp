#pragma once
/**
(C) Copyright 2021-2022 DQ Robotics Developers

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

#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>

namespace DQ_robotics
{

class DQ_SerialManipulatorDenso: public DQ_SerialManipulator
{
protected:
    MatrixXd    denso_matrix_;

    DQ _denso2dh(const double& q, const int& ith) const;
public:

    // Deprecated on 22.04, will be removed on the next release.
    [[deprecated("Use ? instead.")]] VectorXd get_as() const;
    [[deprecated("Use ? instead.")]] VectorXd get_bs() const;
    [[deprecated("Use ? instead.")]] VectorXd get_ds() const;
    [[deprecated("Use ? instead.")]] VectorXd get_alphas() const;
    [[deprecated("Use ? instead.")]] VectorXd get_betas() const;
    [[deprecated("Use ? instead.")]] VectorXd get_gammas() const;

    DQ_SerialManipulatorDenso()=delete;
    DQ_SerialManipulatorDenso(const MatrixXd& denso_matrix);

    using DQ_SerialManipulator::raw_pose_jacobian;
     using DQ_SerialManipulator::raw_pose_jacobian_derivative;
    using DQ_SerialManipulator::raw_fkm;

    MatrixXd raw_pose_jacobian(const VectorXd& q_vec, const int& to_ith_link) const override;
    MatrixXd raw_pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot, const int& to_ith_link) const override;
    DQ raw_fkm(const VectorXd &q_vec, const int &to_ith_link) const override;
};

}
