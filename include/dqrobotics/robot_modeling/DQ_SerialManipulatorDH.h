/**
(C) Copyright 2020 DQ Robotics Developers

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

#ifndef DQ_SerialManipulatorDH_H
#define DQ_SerialManipulatorDH_H

#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>

namespace DQ_robotics
{

class DQ_SerialManipulatorDH: public DQ_SerialManipulator
{
protected:
    MatrixXd    dh_matrix_;

    DQ _get_w(const int& ith) const;
    DQ _dh2dq(const double& q, const int& ith) const;
public:
    // Possible joint types
    enum JOINT_TYPES{ JOINT_ROTATIONAL=0, JOINT_PRISMATIC };

    DQ_SerialManipulatorDH()=delete;
    DQ_SerialManipulatorDH(const MatrixXd& dh_matrix);

    VectorXd get_thetas() const;
    VectorXd get_ds() const;
    VectorXd get_as() const;
    VectorXd get_alphas() const;
    VectorXd get_types() const;

    //Using
    using DQ_SerialManipulator::raw_pose_jacobian;
    using DQ_SerialManipulator::raw_fkm;

    //Override from DQ_SerialManipulator
    MatrixXd raw_pose_jacobian(const VectorXd& q_vec, const int& to_ith_link) const override;
    DQ raw_fkm(const VectorXd &q_vec, const int &to_ith_link) const override;
};

}//Namespace DQRobotics

#endif // DQ_SerialManipulatorDH_H_INCLUDED
