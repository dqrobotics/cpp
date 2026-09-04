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
- Murilo M. Marinho (murilomarinho@ieee.org)
*/

#ifndef DQ_ROBOTICS_ROBOT_MODELING_DQ_MOBILEBASE
#define DQ_ROBOTICS_ROBOT_MODELING_DQ_MOBILEBASE

#include<dqrobotics/DQ.h>
#include<dqrobotics/robot_modeling/DQ_Kinematics.h>

namespace DQ_robotics
{

/**
 * @brief Abstract class that defines an interface for mobile bases.
 *
 * DQ_MobileBase specializes DQ_Kinematics for mobile robots whose pose is
 * typically described by a low-dimensional configuration vector and an
 * additional rigid displacement from the planar base pose to the actual base frame.
 *
 * @note This class remains abstract because the kinematic interface inherited
 * from DQ_Kinematics must be implemented by subclasses.
 *
 * @see DQ_Kinematics, DQ_HolonomicBase
 */
class DQ_MobileBase : public DQ_Kinematics
{
protected:
    /** @brief Constant rigid displacement from the raw mobile-base pose to the base frame. */
    DQ frame_displacement_;

    /** @brief Constructs a mobile base with identity frame displacement. */
    DQ_MobileBase();
public:
    /** @brief Virtual destructor. */
    virtual ~DQ_MobileBase() = default;

    //Abstract methods (Inherited from DQ_Kinematics)
    //virtual int      get_dim_configuration_space() const = 0;
    //virtual DQ       fkm(const VectorXd& joint_configurations) const = 0;
    //virtual MatrixXd pose_jacobian(const VectorXd& joint_configurations,const int& to_link) const = 0;
    //virtual MatrixXd pose_jacobian_derivative(const VectorXd& q, const VectorXd& q_dot, const int& to_link) const = 0;

    /**
     * @brief Sets the rigid displacement from the raw mobile-base pose to the base frame.
     *
     * @param pose Constant rigid displacement represented as a dual quaternion.
     */
    void set_frame_displacement(const DQ& pose);
    /**
     * @brief Returns the rigid displacement from the raw mobile-base pose to the base frame.
     *
     * @return The stored frame displacement.
     */
    DQ   frame_displacement();

};

}

#endif
