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

#ifndef DQ_ROBOT_MODELLING_DQ_KINEMATICS_H
#define DQ_ROBOT_MODELLING_DQ_KINEMATICS_H

#include<dqrobotics/DQ.h>

namespace DQ_robotics
{
class DQ_Kinematics
{
protected:
    DQ reference_frame_;
    DQ base_frame_;
    std::string name_;
    VectorXd q;
    int dim_configuration_space_;

    void _check_to_ith_link(const int& to_ith_link) const;
    void _check_q_vec(const VectorXd& q_vec) const;

    //Constructor
    DQ_Kinematics();
public:
   //Virtual destructor
    virtual ~DQ_Kinematics() = default;

    //Concrete methods
    void set_reference_frame(const DQ& reference_frame);
    DQ   reference_frame() const;
    void set_base_frame(const DQ& base_frame);
    DQ   base_frame() const;
    void set_name(const std::string& name);
    std::string name() const;

    //PURE virtual methods
    virtual DQ       fkm(const VectorXd& joint_configurations) const = 0;
    virtual MatrixXd pose_jacobian(const VectorXd& joint_configurations,const int& to_link) const = 0;
    //Virtual methods
    virtual MatrixXd pose_jacobian(const VectorXd& joint_configurations) const;
    virtual int      get_dim_configuration_space() const;

    //Static methods
    static MatrixXd distance_jacobian(const MatrixXd& pose_jacobian, const DQ& pose);
    static MatrixXd translation_jacobian(const MatrixXd& pose_jacobian, const DQ& pose);
    static MatrixXd rotation_jacobian(const MatrixXd& pose_jacobian);
    static MatrixXd line_jacobian(const MatrixXd& pose_jacobian, const DQ& pose, const DQ& line_direction);
    static MatrixXd plane_jacobian(const MatrixXd& pose_jacobian, const DQ& pose, const DQ& plane_normal);

    static MatrixXd point_to_point_distance_jacobian(const MatrixXd& translation_jacobian, const DQ& robot_point, const DQ& workspace_point);
    static double   point_to_point_residual         (const DQ& robot_point, const DQ& workspace_point, const DQ& workspace_point_derivative);
    static MatrixXd point_to_line_distance_jacobian (const MatrixXd& translation_jacobian, const DQ& robot_point, const DQ& workspace_line);
    static double   point_to_line_residual          (const DQ& robot_point, const DQ& workspace_line, const DQ& workspace_line_derivative);
    static MatrixXd point_to_plane_distance_jacobian(const MatrixXd& translation_jacobian, const DQ& robot_point, const DQ& workspace_plane);
    static double   point_to_plane_residual         (const DQ& translation, const DQ& plane_derivative);
    static MatrixXd line_to_point_distance_jacobian (const MatrixXd& line_jacobian, const DQ& robot_line, const DQ& workspace_point);
    static double   line_to_point_residual          (const DQ& robot_line, const DQ& workspace_point, const DQ& workspace_point_derivative);
    static MatrixXd line_to_line_distance_jacobian  (const MatrixXd& line_jacobian, const DQ& robot_line, const DQ& workspace_line);
    static double   line_to_line_residual           (const DQ& robot_line, const DQ& workspace_line, const DQ& workspace_line_derivative);
    static MatrixXd plane_to_point_distance_jacobian(const MatrixXd& plane_jacobian, const DQ& workspace_point);
    static double   plane_to_point_residual         (const DQ& robot_plane, const DQ& workspace_point_derivative);
};
}

#endif
