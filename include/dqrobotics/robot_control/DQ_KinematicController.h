#pragma once
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

#include <memory>

#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>

namespace DQ_robotics
{

enum ControlObjective
{
    None,
    Distance,
    DistanceToPlane,
    Line,
    Plane,
    Pose,
    Rotation,
    Translation
};

class DQ_KinematicController
{
protected:
    [[deprecated]]
    DQ_Kinematics* robot_;
    std::shared_ptr<DQ_Kinematics> robot_sptr_;
    ControlObjective control_objective_;
    DQ attached_primitive_;
    DQ target_primitive_;

    double gain_;
    double damping_;

    bool system_reached_stable_region_;
    VectorXd last_control_signal_;
    VectorXd last_error_signal_;

    double stability_threshold_;
    int stability_counter_;
    int stability_counter_max_;

    //For backwards compatibility reasons, to be removed
    DQ_Kinematics* _get_robot_ptr() const;

    [[deprecated]]
    DQ_KinematicController(DQ_Kinematics* robot);
    DQ_KinematicController(std::shared_ptr<DQ_Kinematics> robot_sptr);
public:
    DQ_KinematicController()=delete;

    ControlObjective get_control_objective() const;

    MatrixXd get_jacobian(const VectorXd& q) const;

    VectorXd get_task_variable(const VectorXd& q) const;

    VectorXd get_last_error_signal() const;

    bool is_set() const;

    bool system_reached_stable_region() const;

    void set_control_objective(const ControlObjective& control_objective);

    void set_gain(const double& gain);

    void set_damping(const double& damping);

    void set_stability_threshold(const double& threshold);

    void set_primitive_to_effector(const DQ& primitive);

    void set_target_primitive(const DQ& primitive);

    void set_stability_counter_max(const int& max);

    void reset_stability_counter();

    //Virtual
    virtual ~DQ_KinematicController()=default;
    virtual VectorXd compute_setpoint_control_signal(const VectorXd& q, const VectorXd& task_reference)=0;
    virtual VectorXd compute_tracking_control_signal(const VectorXd& q, const VectorXd& task_reference, const VectorXd& feed_forward)=0;
    virtual void verify_stability(const VectorXd& task_error);

};

}

