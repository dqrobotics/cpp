/**
(C) Copyright 2019-2020 DQ Robotics Developers

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

#include<dqrobotics/robot_modeling/DQ_Kinematics.h>
#include<dqrobotics/utils/DQ_Constants.h>
#include<dqrobotics/utils/DQ_Geometry.h>

#include<dqrobotics/internal/_dq_linesegment.h>

namespace DQ_robotics
{

DQ_Kinematics::DQ_Kinematics():
    reference_frame_(1),
    base_frame_(1)
{

}

/**
 * @brief Protected method to check if the index to a link is valid.
 * @param to_ith_link The index to a link.
 * @throws std::runtime_error when the value of  to_ith_link is invalid.
 */
void DQ_Kinematics::_check_to_ith_link(const int &to_ith_link) const
{
    if(to_ith_link >= this->get_dim_configuration_space() || to_ith_link < 0)
    {
        throw std::runtime_error(std::string("Tried to access link index ") + std::to_string(to_ith_link) + std::string(" which is unnavailable."));
    }
}

/**
 * @brief Protected method to check if the size of the vector of joint values is valid.
 * @param q_vec Vector of joint values.
 * @throws std::runtime_error when the size of  q_vec is invalid.
 */
void DQ_Kinematics::_check_q_vec(const VectorXd &q_vec) const
{
    if(q_vec.size() != get_dim_configuration_space())
    {
        throw std::runtime_error(std::string("Input vector must have size ") + std::to_string(get_dim_configuration_space()));
    }
}

/**
 * @brief Sets the reference frame used for the fkm() and pose_jacobian() methods.
 * @param reference_frame A unit dual quaternion representing the reference frame.
 * @throws std::runtime_error when reference_frame is not a unit dual quaternion.
 */
void DQ_Kinematics::set_reference_frame(const DQ &reference_frame)
{
    if(!is_unit(reference_frame))
        throw std::runtime_error("The input to set_reference_frame() must be a unit dual quaternion.");
    reference_frame_ = reference_frame;
}

/**
 * @brief Returns the reference frame set by set_reference_frame().
 * @return A unit dual quaternion representing the reference frame.
 */
DQ   DQ_Kinematics::get_reference_frame() const
{
    return reference_frame_;
}

/**
 * @brief Sets the base frame with respect to the global reference frame (i.e., the identity).
 *        The rigid motion from the global reference frame to the robot base is given by the
 *        unit dual quaternion  base_frame. This function is used to define the 'physical'
 *        place of the robot base and it does not necessarily coincides with the reference
 *        frame.
 *
 * @param base_frame A unit dual quaternion representing the reference frame.
 * @throws std::runtime_error when  base_frame is not a unit dual quaternion.
 */
void DQ_Kinematics::set_base_frame(const DQ &base_frame)
{
    if(!is_unit(base_frame))
        throw std::runtime_error("The input to set_base_frame() must be a unit dual quaternion.");
    base_frame_ = base_frame;
}

/**
 * @brief Returns the base frame set by set_base_frame().
 * @return A unit dual quaternion representing the reference frame.
 */
DQ DQ_Kinematics::get_base_frame() const
{
    return base_frame_;
}

void DQ_Kinematics::set_name(const std::string &name)
{
    name_ = name;
}

std::string DQ_Kinematics::get_name() const
{
    return name_;
}

/**
 * @brief returns the Jacobian that satisfies
          vec8(pose_dot) = J * q_dot, where pose = fkm(), 'pose_dot' is the time
          derivative of the pose and  joint_configurations is the configuration vector.
 * @param joint_configurations The VectorXd representing the joint configurations.
 * @return a MatrixXd representing the desired Jacobian.
 */
MatrixXd DQ_Kinematics::pose_jacobian(const VectorXd &joint_configurations) const
{
    return pose_jacobian(joint_configurations, get_dim_configuration_space()-1);
}

/**
 * @brief returns the Jacobian derivative 'J_dot' that satisfies
          vec8(pose_dot_dot) = J_dot * q_dot + J*q_dot_dot, where pose = fkm(), 'pose_dot' is the time
          derivative of the pose and 'q_dot' represents the robot configuration velocities.
 * @param q The VectorXd representing the robot configurations.
 * @param q_dot The VectorXd representing the robot configuration velocities.
 * @return a MatrixXd representing the desired Jacobian derivative.
 */
MatrixXd DQ_Kinematics::pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot) const
{
    return pose_jacobian_derivative(q, q_dot, get_dim_configuration_space()-1);
}

/**
 * @brief Returns the dimensions of the configuration space.
 * @return An int presenting the dimension of the configuration space.
 */
int DQ_Kinematics::get_dim_configuration_space() const
{
    return dim_configuration_space_;
}

/**
 * @brief Given the  pose_jacobian and the corresponding unit dual
 *        quaternion  pose that satisfy vec8(pose_dot) = pose_jacobian *
 *        q_dot, distance_jacobian() returns the distance
 *        Jacobian; that it, the Jacobian that satisfies the relation
 *        dot(d^2) = Jd * q_dot, where dot(d^2) is the time derivative of
 *        the square of the distance between the origin of the frame
 *        represented by  pose and the origin of the reference frame.
 * @param pose_jacobian The MatrixXd representing the pose Jacobian, as obtained from
 *        pose_jacobian().
 * @param pose The DQ representing the pose related to the pose Jacobian, as obtained from
 *        fkm().
 * @return The MatrixXd representing the desired Jacobian.
 */
MatrixXd DQ_Kinematics::distance_jacobian(const MatrixXd &pose_jacobian, const DQ &pose)
{
    const DQ t        = translation(pose);
    const MatrixXd Jt = DQ_Kinematics::translation_jacobian(pose_jacobian,pose);
    const MatrixXd Jd = 2*vec4(t).transpose()*Jt;
    return Jd;
}

/**
 * @brief Given the Jacobian  pose_jacobian and the corresponding unit dual
 *        quaternion pose that satisfy vec8(pose_dot) = J *
 *        q_dot, translation_jacobian() returns the Jacobian
 *        that satisfies the relation vec4(p_dot) = Jp * q_dot, where p_dot
 *        is the time derivative of the translation quaternion  pose and q_dot
 *        is the time derivative of the configuration vector.
 * @param pose_jacobian The MatrixXd representing the pose Jacobian, as obtained from
 *        pose_jacobian().
 * @param pose The DQ representing the pose related to the pose Jacobian, as obtained from
 *        fkm().
 * @return The MatrixXd representing the desired Jacobian.
 */
MatrixXd DQ_Kinematics::translation_jacobian(const MatrixXd &pose_jacobian, const DQ &pose)
{
    return 2.0*haminus4(conj(P(pose)))*pose_jacobian.block(4,0,4,pose_jacobian.cols())+
            2.0*hamiplus4(D(pose))*C4()*DQ_Kinematics::rotation_jacobian(pose_jacobian);
}

/**
 * @brief  Given the pose_jacobian and the corresponding unit dual
 *         quaternion pose that satisfy vec8(pose_dot) = J *
 *         q_dot, rotation_jacobian() returns the Jacobian Jr that
 *         satisfies vec4(r_dot) = Jr * q_dot, where r_dot is the time
 *         derivative of the rotation quaternion r in pose = r +
 *         DQ.E*(1/2)*p*r and q_dot is the time derivative of the
 *         configuration vector.
 * @param pose_jacobian The MatrixXd representing the pose Jacobian, as obtained from
 *        pose_jacobian().
 * @return the MatrixXd representing the desired Jacobian.
 */
MatrixXd DQ_Kinematics::rotation_jacobian(const MatrixXd &pose_jacobian)
{
    return pose_jacobian.block(0,0,4,pose_jacobian.cols());
}

/**
 * @brief The line Jacobian given the  rotation_jacobian, the  translation_jacobian, the  pose, and a  line_direction.
 * @param rotation_jacobian the current rotation Jacobian.
 * @param translation_jacobian the current translation Jacobian.
 * @param pose the current end-effector pose.
 * @param line_direction the line direction w.r.t. the  pose reference frame. For example using i_, j_, and k_
 * will return the line Jacobian collinear with, respectively, the x-axis, y-axis, and z-axis of  pose.
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
 * @brief The plane Jacobian given the pose_jacobian, the pose, and a plane_normal.
 * @param pose_jacobian The pose Jacobian as obtained from pose_jacobian().
 * @param pose The pose obtained from fkm() corresponding to pose_jacobian().
 * @param plane_normal the plane normal w.r.t. the  pose reference frame. For example using i_, j_, and k_
 * will return the plane Jacobian whose normal is collinear with, respectively, the x-axis, y-axis, and z-axis of  pose.
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


/**
 * @brief Given a line "lz" rigidly attached to the robot, and a workspace line "l", the distance function f(phi)=dot(lz-l, lz-l)
 *        is useful to control the angle "phi" between both lines. line_to_line_angle_jacobian() returns the distance Jacobian Jd that
 *        satisfy vec(f(phi)_dot) = Jd*q_dot + residual.
 * @param line_jacobian The line Jacobian.
 * @param robot_line The line rigidly attached to the robot.
 * @param workspace_line The workspace line. For example: i_, j_, k_.
 * @return The desired distance Jacobian.
 */
MatrixXd DQ_Kinematics::line_to_line_angle_jacobian(const MatrixXd &line_jacobian, const DQ &robot_line, const DQ &workspace_line)
{
    if(! is_line(robot_line))
    {
        throw std::range_error("The argument robot_line has to be a line.");
    }
    if(! is_line(workspace_line))
    {
        throw std::range_error("The argument workspace_line has to be a line.");
    }

    MatrixXd Jl = line_jacobian.block(0,0,4,line_jacobian.cols());
    return 2.0*vec4(robot_line - workspace_line).transpose()*Jl;
}


/**
 * @brief line_to_line_angle_residual() returns the residual term that satisfy vec(f(phi)_dot) = Jd*q_dot + residual, where
 *        f(phi)=dot(lz-l, lz-l) and Jd is the line-to-line-angle-jacobian.
 * @param robot_line The line rigidly attached to the robot.
 * @param workspace_line The workspace line. For example: i_, j_, k_.
 * @param workspace_line_derivative The time derivative of the workspace line.
 * @return The desired residual
 */
double DQ_Kinematics::line_to_line_angle_residual(const DQ& robot_line, const DQ& workspace_line, const DQ& workspace_line_derivative)
{
    if(! is_line(robot_line))
    {
        throw std::range_error("The argument robot_line has to be a line.");
    }
    if(! is_line(workspace_line))
    {
        throw std::range_error("The argument workspace_line has to be a line.");
    }
    DQ result = 2*dot(robot_line-workspace_line, -1.0*workspace_line_derivative);
    return static_cast<double>(result);
}

/**
 * @brief DQ_Kinematics::line_segment_to_line_segment_distance_jacobian
 * Obtains the squared distance Jacobian between two line segments.
 * This method will check the validity of all dual quaternion inputs, but has no way to check
 * the validity of the Jacobians.
 * As mentioned in Section VI of
 * M. M. Marinho, B. V. Adorno, K. Harada and M. Mitsuishi, "Dynamic Active Constraints for
 * Surgical Robots Using Vector-Field Inequalities," in IEEE Transactions on Robotics, vol. 35, no. 5,
 * pp. 1166-1185, Oct. 2019, doi: 10.1109/TRO.2019.2920078.
 * @param line_jacobian The line Jacobian of the robot_line.
 * @param robot_point_1_translation_jacobian the translation jacobian of robot_point_1.
 * @param robot_point_2_translation_jacobian the translation jacobian of robot_point_2.
 * @param robot_line the robot line.
 * @param robot_point_1 the first point delimiting the robot line segment. Must be on robot_line.
 * @param robot_point_2 the second point delimiting the robot line segment. Must be on robot_line.
 * @param workspace_line the workspace line.
 * @param workspace_point_1 the first point deliming the workspace line segment. Must be on workspace_line.
 * @param workspace_point_2 the second point delimiting the workspace line segment. Must be on workspace_line.
 * @return The suitable distance Jacobian for robot_line and workspace_line.
 */
MatrixXd DQ_Kinematics::line_segment_to_line_segment_distance_jacobian(const MatrixXd& line_jacobian,
                                                                       const MatrixXd& robot_point_1_translation_jacobian,
                                                                       const MatrixXd& robot_point_2_translation_jacobian,
                                                                       const DQ& robot_line,
                                                                       const DQ& robot_point_1,
                                                                       const DQ& robot_point_2,
                                                                       const DQ& workspace_line,
                                                                       const DQ& workspace_point_1,
                                                                       const DQ& workspace_point_2)
{
    if(!DQ_Geometry::is_line_segment(robot_line,robot_point_1,robot_point_2))
        throw std::runtime_error("DQ_Kinematics::line_segment_to_line_segment_distance_jacobian::Input line_1, line_1_point_1, "
                                 "and line_1_point_2 must contitute a valid line segment.");
    if(!DQ_Geometry::is_line_segment(workspace_line,workspace_point_1,workspace_point_2))
        throw std::runtime_error("DQ_Kinematics::line_segment_to_line_segment_distance_jacobian::Input line_2, line_2_point_1, "
                                 "and line_2_point_2 must contitute a valid line segment");

    const DQ& l1 = P(robot_line);
    const DQ& l2 = P(workspace_line);


    if( l1 == l2 || l1 == -l2) //TODO, add a tolerance interval
    {
        //In this case, it is impossible to find a single closest point because all points
        //have equal distance between lines
        return DQ_Kinematics::line_to_line_distance_jacobian(line_jacobian, robot_line, workspace_line);
    }
    else
    {
        auto ce = internal::LineSegment::closest_elements_between_line_segments(
                    {robot_line,robot_point_1,robot_point_2},
                    {workspace_line,workspace_point_1,workspace_point_2});

        switch(std::get<0>(std::get<0>(ce)))
        {
        case internal::LineSegment::Element::Line:
        {
            switch(std::get<1>(std::get<0>(ce)))
            {
            case internal::LineSegment::Element::Line:
                return DQ_Kinematics::line_to_line_distance_jacobian(line_jacobian,robot_line,workspace_line);
            case internal::LineSegment::Element::P1:
                return DQ_Kinematics::line_to_point_distance_jacobian(line_jacobian,robot_line,workspace_point_1);
            case internal::LineSegment::Element::P2:
                return DQ_Kinematics::line_to_point_distance_jacobian(line_jacobian,robot_line,workspace_point_2);
            }
            throw std::runtime_error("Unexpected type in DQ_Kinematics::line_segment_to_line_segment_distance_jacobian()");
        }
        case internal::LineSegment::Element::P1:
        {
            switch(std::get<1>(std::get<0>(ce)))
            {
            case internal::LineSegment::Element::Line:
                return DQ_Kinematics::point_to_line_distance_jacobian(robot_point_1_translation_jacobian,robot_point_1,workspace_line);
            case internal::LineSegment::Element::P1:
                return DQ_Kinematics::point_to_point_distance_jacobian(robot_point_1_translation_jacobian,robot_point_1,workspace_point_1);
            case internal::LineSegment::Element::P2:
                return DQ_Kinematics::point_to_point_distance_jacobian(robot_point_1_translation_jacobian,robot_point_1,workspace_point_2);
            }
            throw std::runtime_error("Unexpected type in DQ_Kinematics::line_segment_to_line_segment_distance_jacobian()");
        }
        case internal::LineSegment::Element::P2:
        {
            switch(std::get<1>(std::get<0>(ce)))
            {
            case internal::LineSegment::Element::Line:
                return DQ_Kinematics::point_to_line_distance_jacobian(robot_point_2_translation_jacobian,robot_point_2,workspace_line);
            case internal::LineSegment::Element::P1:
                return DQ_Kinematics::point_to_point_distance_jacobian(robot_point_2_translation_jacobian,robot_point_2,workspace_point_1);
            case internal::LineSegment::Element::P2:
                return DQ_Kinematics::point_to_point_distance_jacobian(robot_point_2_translation_jacobian,robot_point_2,workspace_point_2);
            }
            throw std::runtime_error("Unexpected type in DQ_Kinematics::line_segment_to_line_segment_distance_jacobian()");
        }
        default:
            throw std::runtime_error("Unexpected type in DQ_Kinematics::line_segment_to_line_segment_jacobian()");
        }

    }
    throw std::runtime_error("Unexpected end of method in DQ_Kinematics::line_segment_to_line_segment_jacobian()");
}

}
