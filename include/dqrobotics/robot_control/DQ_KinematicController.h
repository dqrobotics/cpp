#pragma once
/**
(C) Copyright 2019-2024 DQ Robotics Developers

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

    1. Murilo M. Marinho (murilomarinho@ieee.org)
       Responsible for the original implementation

    2. Juan Jose Quiroz Omana
       Fixed bug 59 (https://github.com/dqrobotics/python/issues/59)
            - Initialized variables that are initialized in the Matlab implementation
              of the class DQ_KinematicController.m.
              Specifically, I initialized damping, gain, system_reached_stable_region_,
              stability_threshold, stability_counter, and stability_counter_max.
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
    //Deprecated together with the raw pointer constructors but without the C++14 attribute as it is too noisy.
    DQ_Kinematics* robot_;
    std::shared_ptr<DQ_Kinematics> robot_sptr_;
    ControlObjective control_objective_;
    DQ attached_primitive_;
    DQ target_primitive_;

    double gain_{0};
    double damping_{0};

    bool system_reached_stable_region_{false};
    VectorXd last_control_signal_;
    VectorXd last_error_signal_;

    double stability_threshold_{0};
    int stability_counter_{0};
    int stability_counter_max_{10};

    //For backwards compatibility reasons, to be removed
    DQ_Kinematics* _get_robot_ptr() const;

    std::shared_ptr<DQ_Kinematics> _get_robot() const;

    //Deprecated
    [[deprecated("Use the smart pointer version instead.")]]
    DQ_KinematicController(DQ_Kinematics* robot);
    DQ_KinematicController(const std::shared_ptr<DQ_Kinematics>& robot);
    DQ_KinematicController();
public:    

    ControlObjective get_control_objective() const;

    MatrixXd get_jacobian(const VectorXd& q) const;

    VectorXd get_task_variable(const VectorXd& q) const;

    VectorXd get_last_error_signal() const;

    bool is_set() const;

    bool system_reached_stable_region() const;

    void set_control_objective(const ControlObjective& control_objective);

    void set_gain(const double& gain);
    double get_gain() const;

    void set_damping(const double& damping);
    double get_damping() const;

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

