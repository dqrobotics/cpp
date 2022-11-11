/**
(C) Copyright 2019-2022 DQ Robotics Developers

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

#ifndef DQ_ROBOTICS_ROBOT_MODELING_DQ_DIFFERENTIALDRIVEROBOT
#define DQ_ROBOTICS_ROBOT_MODELING_DQ_DIFFERENTIALDRIVEROBOT

#include<dqrobotics/DQ.h>
#include<dqrobotics/robot_modeling/DQ_HolonomicBase.h>

namespace DQ_robotics
{

class DQ_DifferentialDriveRobot : public DQ_HolonomicBase
{
protected:
    double wheel_radius_;
    double distance_between_wheels_;
public:
    DQ_DifferentialDriveRobot(const double& wheel_radius, const double& distance_between_wheels);

    MatrixXd constraint_jacobian(const double& phi) const;
    MatrixXd constraint_jacobian_derivative(const double& phi, const double& phi_dot) const;
    MatrixXd pose_jacobian(const VectorXd& q, const int& to_link) const override;
    MatrixXd pose_jacobian(const VectorXd &q) const override;
    MatrixXd pose_jacobian_derivative(const VectorXd& q,
                                      const VectorXd& q_dot,
                                      const int& to_link) const override;
    MatrixXd pose_jacobian_derivative (const VectorXd& q,
                                       const VectorXd& q_dot) const override;
};

}

#endif
