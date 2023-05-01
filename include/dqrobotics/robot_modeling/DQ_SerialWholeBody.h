/**
(C) Copyright 2020-2022 DQ Robotics Developers

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
- Murilo M. Marinho (murilomarinho@ieee.org)
*/

#ifndef DQ_ROBOT_MODELLING_DQ_SERIAL_WHOLE_BODY_H
#define DQ_ROBOT_MODELLING_DQ_SERIAL_WHOLE_BODY_H

#include<vector>
#include<memory>
#include<dqrobotics/DQ.h>
#include<dqrobotics/robot_modeling/DQ_Kinematics.h>
#include<dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include<dqrobotics/robot_modeling/DQ_HolonomicBase.h>

namespace DQ_robotics
{

class DQ_SerialWholeBody : public DQ_Kinematics
{
protected:
    std::vector<std::shared_ptr<DQ_Kinematics>> chain_;
    void _check_to_ith_chain(const int& to_ith_chain) const;
    void _check_to_jth_link_of_ith_chain(const int& to_ith_chain, const int& to_jth_link) const;
public:
    DQ_SerialWholeBody()=delete;
    DQ_SerialWholeBody(std::shared_ptr<DQ_Kinematics> robot, const std::string type=std::string("standard"));

    void add(std::shared_ptr<DQ_Kinematics> robot);

    DQ raw_fkm_by_chain(const VectorXd& q, const int& to_ith_chain, const int& to_jth_link) const;
    DQ raw_fkm_by_chain(const VectorXd& q, const int& to_ith_chain) const;
    std::tuple<int, int> get_chain_and_link_from_index(const int& to_ith_link) const;

    DQ raw_fkm(const VectorXd& q) const;
    DQ raw_fkm(const VectorXd& q, const int& to_ith_link) const;

    void set_effector(const DQ& effector);

    DQ_Kinematics* get_chain(const int& to_ith_chain);
    DQ_SerialManipulatorDH get_chain_as_serial_manipulator_dh(const int& to_ith_chain) const;
    DQ_HolonomicBase get_chain_as_holonomic_base(const int& to_ith_chain) const;
    MatrixXd raw_pose_jacobian_by_chain(const VectorXd& q, const int& to_ith_chain, const int& to_jth_link) const;
    MatrixXd raw_pose_jacobian_derivative_by_chain(const VectorXd& q,
                                                   const VectorXd& q_dot,
                                                   const int& to_ith_chain,
                                                   const int& to_jth_link) const; //To be implemented.

    //Abstract methods' implementation
    DQ fkm(const VectorXd& q) const override;
    DQ fkm(const VectorXd&, const int& to_ith_link) const override;
    MatrixXd pose_jacobian(const VectorXd& q, const int& to_ith_link) const override;
    MatrixXd pose_jacobian(const VectorXd& q) const override;
    MatrixXd pose_jacobian_derivative(const VectorXd& q,
                                      const VectorXd& q_dot,
                                      const int& to_ith_link) const override; //To be implemented.
    MatrixXd pose_jacobian_derivative (const VectorXd& q,
                                       const VectorXd& q_dot) const override; //To be implemented.
};

}

#endif
