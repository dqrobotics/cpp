#pragma once
/**
(C) Copyright 2022 DQ Robotics Developers

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
- Juan Jose Quiroz Omana -  juanjqo@g.ecc.u-tokyo.ac.jp
*/

#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>

namespace DQ_robotics
{

class DQ_SerialManipulatorMDH: public DQ_SerialManipulator
{
protected:
    MatrixXd    mdh_matrix_;

    DQ _get_w(const int& ith) const;
    DQ _mdh2dq(const double& q, const int& ith) const;
public:

    // Deprecated on 22.04, will be removed on the next release.
    enum [[deprecated("Use ? instead.")]] JOINT_TYPES{ JOINT_ROTATIONAL=0, JOINT_PRISMATIC };
    [[deprecated("Use ? instead.")]] VectorXd get_thetas() const;
    [[deprecated("Use ? instead.")]] VectorXd get_ds() const;
    [[deprecated("Use ? instead.")]] VectorXd get_as() const;
    [[deprecated("Use ? instead.")]] VectorXd get_alphas() const;
    [[deprecated("Use ? instead.")]] VectorXd get_types() const;

    DQ_SerialManipulatorMDH()=delete;
    DQ_SerialManipulatorMDH(const MatrixXd& mdh_matrix);    

    MatrixXd pose_jacobian_derivative(const VectorXd& q_vec, const VectorXd& q_vec_dot, const int& to_ith_link) const;
    MatrixXd pose_jacobian_derivative(const VectorXd& q_vec, const VectorXd& q_vec_dot) const;

    using DQ_SerialManipulator::raw_pose_jacobian;
    using DQ_SerialManipulator::raw_fkm;

    MatrixXd raw_pose_jacobian(const VectorXd& q_vec, const int& to_ith_link) const override;
    DQ raw_fkm(const VectorXd &q_vec, const int &to_ith_link) const override;
};

}


