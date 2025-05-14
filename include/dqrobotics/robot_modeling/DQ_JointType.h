/**
(C) Copyright 2011-2025 DQ Robotics Developers

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
1. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
    - Responsible for the original implementation.
*/


#pragma once

namespace DQ_robotics
{
class DQ_JointType
{
public:
    enum JOINT_TYPE{
        REVOLUTE    = 0,
        PRISMATIC,
        SPHERICAL,
        CYLINDRICAL,
        PLANAR,
        SIX_DOF,
        HELICAL
    };
    // This definition enables switch cases and comparisons.
    constexpr operator JOINT_TYPE() const { return joint_type_; }
private:
    JOINT_TYPE joint_type_;

public:
    /**
     * @brief DQ_JointType Default constructor method.
     */
    DQ_JointType() = default;

    /**
     * @brief DQ_JointType Constructor method
     * @param joint_type The joint type. Example: REVOLUTE, PRISMATIC,
     *                   SPHERICAL, CYLINDRICAL, PLANAR, SIX_DOF, or HELICAL.
     */
    DQ_JointType(const JOINT_TYPE& joint_type): joint_type_{joint_type}{};
};

}
