#pragma once
//This is an implementation of the controller described in the following work:
//S. Chiaverini,
//"Singularity-robust task-priority redundancy resolution for real-time kinematic control of robot manipulators,"
//in IEEE Transactions on Robotics and Automation,
//vol. 13, no. 3, pp. 398-410, June 1997,
//doi: 10.1109/70.585902.
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
- Murilo M. Marinho (murilo@g.ecc.u-tokyo.ac.jp)
*/
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_control/DQ_PseudoinverseController.h>

namespace DQ_robotics
{

class DQ_NumericalFilteredPseudoinverseController: public DQ_PseudoinverseController
{
protected:
    double epsilon_; //Size of the singular region, described on the text after Eq. (15)
    double lambda_max_; //Maximum value for the numerical filtered damping, described on the text after Eq. (15)
    //double damping_; //(Member variable of DQ_KinematicController: Isotropic damping described on the text above Eq. (20)

    //log
    MatrixXd last_filtered_damping_;
    double last_jacobian_rank_;
    std::tuple<MatrixXd,MatrixXd,MatrixXd> last_jacobian_svd_;
public:
    DQ_NumericalFilteredPseudoinverseController() = delete;
    [[deprecated("Use the smart pointer version instead")]]
    DQ_NumericalFilteredPseudoinverseController(DQ_Kinematics* robot);
    DQ_NumericalFilteredPseudoinverseController(const std::shared_ptr<DQ_Kinematics>& robot);

    VectorXd compute_setpoint_control_signal(const VectorXd& q, const VectorXd& task_reference) override;
    VectorXd compute_tracking_control_signal(const VectorXd& q, const VectorXd& task_reference, const VectorXd& feed_forward) override;

    void set_maximum_numerical_filtered_damping(const double& numerical_filtered_damping);
    void set_singular_region_size(const double& singular_region_size);

    double get_maximum_numerical_filtered_damping() const;
    double get_singular_region_size() const;
    MatrixXd get_last_filtered_damping() const;
    int get_last_jacobian_rank() const;
    std::tuple<MatrixXd,MatrixXd,MatrixXd> get_last_jacobian_svd() const;
};

}
