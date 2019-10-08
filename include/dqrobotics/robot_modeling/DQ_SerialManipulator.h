/**
(C) Copyright 2011-2018 DQ Robotics Developers

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
- Murilo M. Marinho        (murilo@nml.t.u-tokyo.ac.jp)
- Mateus Rodrigues Martins (martinsrmateus@gmail.com)
*/

#ifndef DQ_SerialManipulator_H
#define DQ_SerialManipulator_H

#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>

using namespace Eigen;

namespace DQ_robotics
{

class DQ_SerialManipulator: public DQ_Kinematics
{
private:
    MatrixXd    dh_matrix_;
    std::string dh_matrix_convention_;

    VectorXd lower_q_limit_;
    VectorXd upper_q_limit_;

    DQ curr_effector_;

    DQ _dh2dq(const double& theta, const int& to_ith_link) const;
    DQ _get_z(const DQ& h) const;

    void _check_to_ith_link(const int& to_ith_link) const;
    void _check_q_vec(const VectorXd& q_vec) const;
public:
    DQ_SerialManipulator() = delete;
    DQ_SerialManipulator(const MatrixXd& dh_matrix, const std::string& convention = "standard");

    MatrixXd getDHMatrix();
    VectorXd theta() const;
    VectorXd d() const;
    VectorXd a() const;
    VectorXd alpha() const;
    std::string convention() const;

    VectorXd dummy() const;
    void set_dummy( const VectorXd& dummy_vector);
    int n_dummy() const;

    void set_lower_q_limit(const VectorXd& lower_q_limit);
    VectorXd lower_q_limit() const;

    void set_upper_q_limit(const VectorXd& upper_q_limit);
    VectorXd upper_q_limit() const;

    DQ effector() const;
    DQ set_effector( const DQ& new_effector);

    DQ raw_fkm( const VectorXd& q_vec) const;
    DQ raw_fkm( const VectorXd& q_vec, const int& to_ith_link) const;

    DQ fkm( const VectorXd& q_vec) const override; //Override from DQ_Kinematics
    DQ fkm( const VectorXd& q_vec, const int& to_ith_link) const;

    MatrixXd raw_pose_jacobian (const VectorXd& q_vec) const;
    MatrixXd raw_pose_jacobian ( const VectorXd& q_vec, const int& to_ith_link) const;

    MatrixXd pose_jacobian_derivative( const VectorXd& q_vec, const VectorXd& q_vec_dot) const;
    MatrixXd pose_jacobian_derivative( const VectorXd& q_vec, const VectorXd& q_vec_dot, const int& to_ith_link) const;

    int get_dim_configuration_space() const override; //Override from DQ_Kinematics

    MatrixXd pose_jacobian(const VectorXd& q_vec, const int& to_ith_link) const override; //Override from DQ_Kinematics
    MatrixXd pose_jacobian(const VectorXd& q_vec) const override; //Override from DQ_Kinematics

};

}//Namespace DQRobotics

#endif // DQ_SerialManipulator_H_INCLUDED
