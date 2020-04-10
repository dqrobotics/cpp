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
    DQ _dh2dq_dot(const double& q, const int& ith) const;

    //Override from DQ_SerialManipulator
    DQ _dh2dq(const double& q, const int& ith) const override;

public:
    // Possible joint types
    enum JOINT_TYPES{ JOINT_ROTATIONAL=0, JOINT_PRISMATIC };

    DQ_SerialManipulatorDH()=delete;
    DQ_SerialManipulatorDH(const MatrixXd& dh_matrix, const std::string& convention = "standard");

    VectorXd type() const;

    //Override from DQ_SerialManipulator
    MatrixXd raw_pose_jacobian(const VectorXd& q_vec, const int& to_ith_link) const override;
    MatrixXd pose_jacobian(const VectorXd& q_vec, const int& to_ith_link) const override;
    MatrixXd pose_jacobian(const VectorXd& q_vec) const override;
    DQ fkm(const VectorXd& q_vec) const override;
    int get_dim_configuration_space() const override;

    //Use as in DQ_SerialManipulator
    MatrixXd getDHMatrix();
    VectorXd theta() const;
    VectorXd d() const;
    VectorXd a() const;
    VectorXd alpha() const;
    std::string convention() const;
    void set_lower_q_limit(const VectorXd& lower_q_limit);
    VectorXd lower_q_limit() const;
    void set_upper_q_limit(const VectorXd& upper_q_limit);
    VectorXd upper_q_limit() const;
    DQ effector() const;
    DQ set_effector(const DQ& new_effector);
    MatrixXd raw_pose_jacobian(const VectorXd& q_vec) const;
    DQ raw_fkm(const VectorXd& q_vec) const;
    DQ raw_fkm(const VectorXd& q_vec, const int& to_ith_link) const;
    DQ fkm(const VectorXd& q_vec, const int& to_ith_link) const;
};

}//Namespace DQRobotics

#endif // DQ_SerialManipulatorDH_H_INCLUDED
