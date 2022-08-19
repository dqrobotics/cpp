/**
(C) Copyright 2019 DQ Robotics Developers

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

#include <dqrobotics/robot_control/DQ_KinematicController.h>
#include <stdexcept>
#include <dqrobotics/utils/DQ_Geometry.h>

namespace DQ_robotics
{

DQ_Kinematics *DQ_KinematicController::_get_robot_ptr() const
{
    return robot_sptr_ ? robot_sptr_.get() : robot_;
}

DQ_KinematicController::DQ_KinematicController(DQ_Kinematics* robot):
    DQ_KinematicController()
{
    robot_ = robot;
}

DQ_KinematicController::DQ_KinematicController(const std::shared_ptr<DQ_Kinematics> &robot):
    DQ_KinematicController()
{
    robot_sptr_ = robot;
}

DQ_KinematicController::DQ_KinematicController():
    robot_(nullptr),
    control_objective_(ControlObjective::None),
    gain_(0.0),
    system_reached_stable_region_(false),
    last_error_signal_(VectorXd::Zero(1)),//Todo: change this inialization to use empty vector
    last_control_signal_(VectorXd::Zero(1)),//Todo: change this inialization to use empty vector
    stability_threshold_(0.0),
    stability_counter_(0.0),
    stability_counter_max_(10.0),
    attached_primitive_(0.0),
    target_primitive_(0.0)
{

}

void DQ_KinematicController::verify_stability(const VectorXd& task_error)
{
    if((last_error_signal_-task_error).norm() < stability_threshold_)
    {
        if(stability_counter_ < stability_counter_max_)
            stability_counter_++;
    }
    else
    {
        reset_stability_counter();
    }

    if(stability_counter_ >= stability_counter_max_)
    {
        system_reached_stable_region_ = true;
    }
}

ControlObjective DQ_KinematicController::get_control_objective() const
{
    return control_objective_;
}

VectorXd DQ_KinematicController::get_last_error_signal() const
{
    return last_error_signal_;
}

MatrixXd DQ_KinematicController::get_jacobian(const VectorXd &q) const
{
    DQ_Kinematics* robot_local =_get_robot_ptr();

    if(q.size() != robot_local->get_dim_configuration_space())
        throw std::runtime_error("Calling get_jacobian with an incorrect number of joints " + std::to_string(q.size()));

    const MatrixXd J_pose = robot_local->pose_jacobian(q);
    const DQ       x_pose = robot_local->fkm(q);

    switch(control_objective_)
    {
    case ControlObjective::None:
        throw std::runtime_error("The control objective must be initialized with set_control_objective()");

    case ControlObjective::Distance:
        return DQ_Kinematics::distance_jacobian(J_pose,x_pose);

    case ControlObjective::DistanceToPlane:
    {
        if(!is_plane(target_primitive_))
        {
            throw std::runtime_error("Please set the target plane with the method set_target_primitive()");
        }
        MatrixXd Jt = robot_local->translation_jacobian(J_pose, x_pose);
        DQ t = translation(x_pose);
        return robot_local->point_to_plane_distance_jacobian(Jt, t, target_primitive_);
    }

    case ControlObjective::Line:
        return DQ_Kinematics::line_jacobian(J_pose,x_pose,attached_primitive_);

    case ControlObjective::Plane:
        return DQ_Kinematics::plane_jacobian(J_pose,x_pose,attached_primitive_);

    case ControlObjective::Rotation:
        return DQ_Kinematics::rotation_jacobian(J_pose);

    case ControlObjective::Translation:
        return DQ_Kinematics::translation_jacobian(J_pose,x_pose);

    case ControlObjective::Pose:
        return J_pose;
    }

    //The only way I found to fix both possible warnings of either having a default in the switch or not having the default.
    throw std::runtime_error("Unknown ControlObjective");
}

VectorXd DQ_KinematicController::get_task_variable(const VectorXd &q) const
{
    DQ_Kinematics* robot_local =_get_robot_ptr();

    if(q.size() != robot_local->get_dim_configuration_space())
        throw std::runtime_error("Calling get_task_variable with an incorrect number of joints " + std::to_string(q.size()));

    const DQ x_pose = robot_local->fkm(q);

    switch(control_objective_)
    {
    case ControlObjective::None:
        throw std::runtime_error("The control objective must be initialized with set_control_objective()");

    case ControlObjective::Distance:
    {
        VectorXd p = vec4(translation(x_pose));
        return p.transpose()*p;
    }

    case ControlObjective::DistanceToPlane:
    {
        if(!is_plane(target_primitive_))
        {
            throw std::runtime_error("Set the target plane with the method set_target_primitive()");
        }
        DQ t = translation(x_pose);
        VectorXd distance(1);
        distance(0)=DQ_Geometry::point_to_plane_distance(t, target_primitive_);
        return distance;
    }

    case ControlObjective::Line:
        return vec8(Ad(x_pose,attached_primitive_));

    case ControlObjective::Plane:
        return vec8(Adsharp(x_pose,attached_primitive_));

    case ControlObjective::Rotation:
        return vec4(rotation(x_pose));

    case ControlObjective::Translation:
        return vec4(translation(x_pose));

    case ControlObjective::Pose:
        return vec8(x_pose);
    }

    //The only way I found to fix both possible warnings of either having a default in the switch or not having the default.
    throw std::runtime_error("Unknown ControlObjective");
}

bool DQ_KinematicController::is_set() const
{
    if(control_objective_==ControlObjective::None)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool DQ_KinematicController::system_reached_stable_region() const
{
    return system_reached_stable_region_;
}

void DQ_KinematicController::set_control_objective(const ControlObjective &control_objective)
{
    control_objective_ = control_objective;

    switch(control_objective)
    {
    case ControlObjective::Distance: //This was intentional https://en.cppreference.com/w/cpp/language/attributes/fallthrough
    case ControlObjective::DistanceToPlane:
        last_error_signal_ = VectorXd::Zero(1);
        break;
    case ControlObjective::Line: //This was intentional https://en.cppreference.com/w/cpp/language/attributes/fallthrough
    case ControlObjective::Plane: //This was intentional https://en.cppreference.com/w/cpp/language/attributes/fallthrough
    case ControlObjective::Pose:
        last_error_signal_ = VectorXd::Zero(8);
        break;
    case ControlObjective::Rotation:
    case ControlObjective::Translation:
        last_error_signal_ = VectorXd::Zero(4);
        break;
    case ControlObjective::None:
        break;
    }

}

void DQ_KinematicController::set_gain(const double& gain)
{
    gain_ = gain;
}

void DQ_KinematicController::set_stability_threshold(const double &threshold)
{
    stability_threshold_ = threshold;
}

void DQ_KinematicController::set_primitive_to_effector(const DQ &primitive)
{
    attached_primitive_ = primitive;
}

void DQ_KinematicController::set_target_primitive(const DQ &primitive)
{
    target_primitive_ = primitive;
}

void DQ_KinematicController::set_damping(const double &damping)
{
    damping_ = damping;
}

void DQ_KinematicController::set_stability_counter_max(const int &max)
{
    stability_counter_max_ = max;
}

void DQ_KinematicController::reset_stability_counter()
{
    stability_counter_ = 0;
    system_reached_stable_region_ = false;
}

}
