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
    along with DQ Robotics.  If !, see <http://www.gnu.org/licenses/>.

Contributors:
- Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
*/

#include<dqrobotics/robot_modeling/DQ_Kinematics.h>
#include<dqrobotics/utils/DQ_Constants.h>

namespace DQ_robotics
{

/* **********************************************************************
 *  CONSTRUCTOR
 * *********************************************************************/
DQ_Kinematics::DQ_Kinematics()
{
    reference_frame_ = DQ(1);
    base_frame_      = DQ(1);
}

/* **********************************************************************
 *  CONCRETE METHODS
 * *********************************************************************/

void DQ_Kinematics::set_reference_frame(const DQ &reference_frame)
{
    reference_frame_ = reference_frame;
}

DQ   DQ_Kinematics::reference_frame() const
{
    return reference_frame_;
}

void DQ_Kinematics::set_base_frame(const DQ &base_frame)
{
    base_frame_ = base_frame;
}

DQ DQ_Kinematics::base_frame() const
{
    return base_frame_;
}

void DQ_Kinematics::set_name(const std::string &name)
{
    name_ = name;
}

std::string DQ_Kinematics::name() const
{
    return name_;
}

/* **********************************************************************
 *  STATIC METHODS
 * *********************************************************************/

MatrixXd DQ_Kinematics::distance_jacobian(const MatrixXd &pose_jacobian, const DQ &pose)
{
    const DQ t        = translation(pose);
    const MatrixXd Jt = DQ_Kinematics::translation_jacobian(pose_jacobian,pose);
    const MatrixXd Jd = 2*vec4(t).transpose()*Jt;
    return Jd;
}

MatrixXd DQ_Kinematics::translation_jacobian(const MatrixXd &pose_jacobian, const DQ &pose)
{
    return 2.0*haminus4(conj(P(pose)))*pose_jacobian.block(4,0,4,pose_jacobian.cols())+2.0*hamiplus4(D(pose))*C4()*DQ_Kinematics::rotation_jacobian(pose_jacobian);
}


MatrixXd DQ_Kinematics::rotation_jacobian(const MatrixXd &pose_jacobian)
{
    return pose_jacobian.block(0,0,4,pose_jacobian.cols());
}

/**
 * @brief The line Jacobian given the \p rotation_jacobian, the \p translation_jacobian, the \p pose, and a \p line_direction
 * @param rotation_jacobian the current rotation Jacobian \see rotation_jacobian()
 * @param translation_jacobian the current translation Jacobian \see translation_jacobian()
 * @param pose the current end-effector pose \see fkm()
 * @param line_direction the line direction w.r.t. the \p pose reference frame. For example using i_, j_, and k_
 * will return the line Jacobian collinear with, respectively, the x-axis, y-axis, and z-axis of \p pose,
 */
MatrixXd DQ_Kinematics::line_jacobian(const MatrixXd& pose_jacobian, const DQ& pose, const DQ& line_direction)
{
    /// Aliases
    const DQ&       x  = pose;

    /// Requirements
    const MatrixXd Jt = translation_jacobian(pose_jacobian,pose);
    const MatrixXd Jr = rotation_jacobian(pose_jacobian);

    ///Translation
    const DQ xt = translation(x);

    ///Rotation and Rotation Jacobian
    const DQ xr = rotation(x);

    ///Line direction w.r.t. base
    const DQ l = xr*(line_direction)*conj(xr);

    ///Line direction and moment Jacobians
    const MatrixXd Jrx = (haminus4(line_direction*conj(xr)) + hamiplus4(xr*line_direction)*C4())*Jr;
    const MatrixXd Jmx = crossmatrix4(l).transpose()*Jt + crossmatrix4(xt)*Jrx;

    ///Line Jacobian
    MatrixXd Jlx(8,Jr.cols());
    Jlx << Jrx,Jmx;
    return Jlx;
}

/**
 * @brief The plane Jacobian given the \p rotation_jacobian, the \p translation_jacobian, the \p pose, and a \p plane_normal
 * @param rotation_jacobian the current rotation Jacobian \see rotation_jacobian()
 * @param translation_jacobian the current translation Jacobian \see translation_jacobian()
 * @param pose the current end-effector pose \see fkm()
 * @param plane_normal the plane normal w.r.t. the \p pose reference frame. For example using i_, j_, and k_
 * will return the plane Jacobian whose normal is collinear with, respectively, the x-axis, y-axis, and z-axis of \p pose,
 */
MatrixXd DQ_Kinematics::plane_jacobian(const MatrixXd& pose_jacobian, const DQ& pose, const DQ& plane_normal)
{
    /// Aliases
    const DQ&       x  = pose;

    ///Requirements
    const DQ        xt = translation(x);
    const DQ        xr = rotation(x);
    const MatrixXd  Jr = rotation_jacobian(pose_jacobian);
    const MatrixXd  Jt = translation_jacobian(pose_jacobian,pose);

    ///Plane normal w.r.t base
    const DQ nz = xr*(plane_normal)*conj(xr);

    ///Plane normal Jacobian
    const MatrixXd Jnz = (haminus4(plane_normal*conj(xr)) + hamiplus4(xr*plane_normal)*C4())*Jr;

    ///Plane distance Jacobian
    const MatrixXd Jdz  = (vec4(nz).transpose()*Jt+vec4(xt).transpose()*Jnz);

    ///Plane Jacobian
    MatrixXd JPI = MatrixXd::Zero(8,Jt.cols());
    JPI << Jnz,Jdz,MatrixXd::Zero(3,Jdz.cols());
    return JPI;
}

MatrixXd DQ_Kinematics::point_to_point_distance_jacobian(const MatrixXd& translation_jacobian, const DQ& robot_point, const DQ& workspace_point)
{
    if(! is_pure_quaternion(robot_point))
    {
        throw std::range_error("The argument robot_point has to be a pure quaternion.");
    }
    if(! is_pure_quaternion(workspace_point))
    {
        throw std::range_error("The argument workspace_point has to be a pure quaternion.");
    }

    return 2*vec4(robot_point-workspace_point).transpose()*translation_jacobian;
}

double   DQ_Kinematics::point_to_point_residual         (const DQ& robot_point, const DQ& workspace_point, const DQ& workspace_point_derivative)
{
    if(! is_pure_quaternion(robot_point))
    {
        throw std::range_error("The argument robot_point has to be a pure quaternion.");
    }
    if(! is_pure_quaternion(workspace_point))
    {
        throw std::range_error("The argument workspace_point has to be a pure quaternion.");
    }

    DQ result = 2.0*dot(robot_point-workspace_point,-1.0*workspace_point_derivative);

    return static_cast<double>(result);
}

MatrixXd DQ_Kinematics::point_to_line_distance_jacobian(const MatrixXd& translation_jacobian, const DQ& robot_point, const DQ& workspace_line)
{
    if(! is_pure_quaternion(robot_point))
    {
        throw std::range_error("The argument robot_point has to be a pure quaternion.");
    }
    if(! is_line(workspace_line))
    {
        throw std::range_error("The argument workspace_line has to be a line.");
    }

    const DQ& t = robot_point;

    const DQ l = P(workspace_line);
    const DQ m = D(workspace_line);

    return 2.0*vec4( cross(t,l)-m ).transpose()*crossmatrix4(l).transpose()*translation_jacobian;
}

double   DQ_Kinematics::point_to_line_residual(const DQ& robot_point, const DQ& workspace_line, const DQ& workspace_line_derivative)
{
    if(! is_pure_quaternion(robot_point))
    {
        throw std::range_error("The argument robot_point has to be a pure quaternion.");
    }
    if(! is_line(workspace_line))
    {
        throw std::range_error("The argument workspace_line has to be a line.");
    }

    const DQ& t = robot_point;

    const DQ& l = P(workspace_line);
    const DQ& m = D(workspace_line);
    const DQ& l_dot = P(workspace_line_derivative);
    const DQ& m_dot = D(workspace_line_derivative);

    DQ result = 2.0*dot( cross(t,l_dot) - m_dot , cross(t,l) - m );

    return static_cast<double>(result);
}

MatrixXd DQ_Kinematics::point_to_plane_distance_jacobian(const MatrixXd& translation_jacobian, const DQ& robot_point, const DQ& workspace_plane)
{
    if(! is_pure_quaternion(robot_point))
    {
        throw std::range_error("The argument robot_point has to be a pure quaternion.");
    }
    if(! is_plane(workspace_plane))
    {
        throw std::range_error("The argument workspace_plane has to be a plane.");
    }

    const DQ n = P(workspace_plane);

    return vec4(n).transpose()*translation_jacobian;
}

double DQ_Kinematics::point_to_plane_residual(const DQ& translation, const DQ& plane_derivative)
{
    if(! is_pure_quaternion(translation))
    {
        throw std::range_error("The argument translation has to be a pure quaternion.");
    }

    const DQ& t    = translation;
    const DQ n_dot = P(plane_derivative);
    const DQ d_dot = D(plane_derivative);

    DQ result = dot(t,n_dot) - d_dot;
    return static_cast<double>(result);
}

MatrixXd DQ_Kinematics::line_to_point_distance_jacobian (const MatrixXd& line_jacobian, const DQ& robot_line, const DQ& workspace_point)
{
    if(! is_line(robot_line))
    {
        throw std::range_error("The argument robot_line has to be a line.");
    }
    if(! is_pure_quaternion(workspace_point))
    {
        throw std::range_error("The argument workspace_point has to be a pure quaternion");
    }

    MatrixXd Jl = line_jacobian.block(0,0,4,line_jacobian.cols());
    MatrixXd Jm = line_jacobian.block(4,0,4,line_jacobian.cols());

    // Extract line quaternions
    DQ l = P(robot_line);
    DQ m = D(robot_line);

    return 2.0*vec4(  cross(workspace_point,l)-m ).transpose()*(crossmatrix4(workspace_point)*Jl-Jm);
}

double   DQ_Kinematics::line_to_point_residual(const DQ& robot_line, const DQ& workspace_point, const DQ& workspace_point_derivative)
{
    if(! is_line(robot_line))
    {
        throw std::range_error("The argument robot_line has to be a line.");
    }
    if(! is_pure_quaternion(workspace_point))
    {
        throw std::range_error("The argument workspace_point has to be a pure quaternion");
    }

    // Extract line quaternions
    DQ l = P(robot_line);
    DQ m = D(robot_line);

    // !ational simplicity
    DQ hc1 = cross(workspace_point,l)-m;
    DQ hc2 = cross(workspace_point_derivative,l);

    DQ result = 2.0*dot(hc2,hc1);
    return static_cast<double>(result);
}

MatrixXd DQ_Kinematics::line_to_line_distance_jacobian(const MatrixXd& line_jacobian, const DQ& robot_line, const DQ& workspace_line)
{
    if(! is_line(robot_line))
    {
        throw std::range_error("The argument robot_line has to be a line.");
    }
    if(! is_line(workspace_line))
    {
        throw std::range_error("The argument workspace_line has to be a line.");
    }

    const int DOFS  = line_jacobian.cols();
    const DQ& l_dq  = workspace_line;

    ///Dot product dual part square norm
    //Dot product Jacobian
    const MatrixXd Jdot     = -0.5*(hamiplus8(l_dq)+haminus8(l_dq))*line_jacobian;
    const MatrixXd Jdotdual = Jdot.block(4,0,4,DOFS);
    //Norm Jacobian
    const DQ Plzldot             = P(dot(robot_line,l_dq));
    const DQ Dlzldot             = D(dot(robot_line,l_dq));
    const MatrixXd Jnormdotdual  = 2*vec4(Dlzldot).transpose()*Jdotdual;

    ///Cross product primary part square norm
    //Cross product Jacobian
    const MatrixXd Jcross        = 0.5*(haminus8(l_dq)-hamiplus8(l_dq))*line_jacobian;
    const MatrixXd Jcrossprimary = Jcross.block(0,0,4,DOFS);
    const MatrixXd Jcrossdual    = Jcross.block(5,8,4,DOFS);
    //Norm Jacobian
    const DQ Plzlcross                = P(cross(robot_line,l_dq));
    const DQ Dlzlcross                = D(cross(robot_line,l_dq));
    const MatrixXd Jnormcrossprimary  = 2*vec4(Plzlcross).transpose()*Jcrossprimary;

    /// TODO, check input to acos to be sure it won't be negative.
    /// that happens sometimes in CPP due to rounding errors.
    const double phi = acos(static_cast<double>(Plzldot));

    /// TODO, add a threshold because this will never be zero.
    if( fmod(phi,pi) != 0.0)
    {
        ///Distance Jacobian
        // a
        const double a_temp = vec4(Plzlcross).norm();
        const double a = (1.0)/(a_temp*a_temp);
        // b
        const double b_temp = vec4(Dlzldot).norm();
        const double b = -((b_temp*b_temp)/(a_temp*a_temp*a_temp*a_temp));

        ///Robot line--line squared distance Jacobian
        return a*Jnormdotdual+b*Jnormcrossprimary;
    }
    else
    {
        return 2.0*vec4(Dlzlcross).transpose()*Jcrossdual;
    }

}

double   DQ_Kinematics::line_to_line_residual(const DQ& robot_line, const DQ& workspace_line, const DQ& workspace_line_derivative)
{
    if(! is_line(robot_line))
    {
        throw std::range_error("The argument robot_line has to be a line.");
    }
    if(! is_line(workspace_line))
    {
        throw std::range_error("The argument workspace_line has to be a line.");
    }

    const DQ& l_dq_dot = workspace_line_derivative;
    const DQ& l_dq     = workspace_line;

    //Dot product residual
    const DQ zetadotdual         = D(dot(robot_line,l_dq_dot));
    //Norm Jacobian
    const DQ Plzldot             = P(dot(robot_line,l_dq));
    const DQ Dlzldot             = D(dot(robot_line,l_dq));
    const double zetanormdotdual = 2*vec4(Dlzldot).transpose()*vec4(zetadotdual);

    //Cross product residual
    const DQ zetacrossprimary    = P(cross(robot_line,l_dq_dot));
    const DQ zetacrossdual       = D(cross(robot_line,l_dq_dot));
    //Norm Jacobian
    const DQ Plzlcross                = P(cross(robot_line,l_dq));
    const DQ Dlzlcross                = D(cross(robot_line,l_dq));
    const double zetanormcrossprimary = 2*vec4(Plzlcross).transpose()*vec4(zetacrossprimary);

    /// TODO, check input to acos to be sure it won't be negative.
    /// that happens sometimes in CPP due to rounding errors.
    const double phi = acos(static_cast<double>(Plzldot));

    /// TODO, add a threshold because this will never be zero.
    if( fmod(phi,pi) != 0.0)
    {
        // a
        const double a_temp = vec4(Plzlcross).norm();
        const double a = (1.0)/(a_temp*a_temp);
        // b
        const double b_temp = vec4(Dlzldot).norm();
        const double b = -((b_temp*b_temp)/(a_temp*a_temp*a_temp*a_temp));
        return a*zetanormdotdual+b*zetanormcrossprimary;
    }
    else
    {
        return 2.0*vec4(Dlzlcross).transpose()*vec4(zetacrossdual);
    }
}

MatrixXd DQ_Kinematics::plane_to_point_distance_jacobian(const MatrixXd& plane_jacobian, const DQ& workspace_point)
{
    if(! is_pure_quaternion(workspace_point))
    {
        throw std::range_error("The argument workspace_point has to be a pure quaternion.");
    }

    // Break Jpi into blocks
    MatrixXd Jnz = plane_jacobian.block(0,0,4,plane_jacobian.cols());
    MatrixXd Jdz = plane_jacobian.block(4,0,1,plane_jacobian.cols());

    // Plane distance Jacobian
    return vec4(workspace_point).transpose()*Jnz-Jdz;
}

double   DQ_Kinematics::plane_to_point_residual(const DQ& robot_plane, const DQ& workspace_point_derivative)
{
    if(! is_pure_quaternion(workspace_point_derivative))
    {
        throw std::range_error("The argument workspace_point_derivative has to be a pure quaternion.");
    }

    DQ n_pi = P(robot_plane);

    DQ result = dot(workspace_point_derivative,n_pi);
    return static_cast<double>(result);
}

}
