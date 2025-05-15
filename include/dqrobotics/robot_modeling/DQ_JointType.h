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
#include <string>

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

    /**
     * @brief DQ_JointType Constructor method that allows integer arguments
     * @param joint_type The joint type.
     */
    DQ_JointType(const int& joint_type){
        switch (joint_type) {
        case 0:
            joint_type_ = REVOLUTE;
            break;
        case 1:
            joint_type_ = PRISMATIC;
            break;
        case 2:
            joint_type_ = SPHERICAL;
            break;
        case 3:
            joint_type_ = CYLINDRICAL;
            break;
        case 4:
            joint_type_ = PLANAR;
            break;
        case 5:
            joint_type_ = SIX_DOF;
            break;
        case 6:
            joint_type_ = HELICAL;
            break;
        default:
            throw std::runtime_error("Invalid joint type");
        }
    }

    /**
     * @brief ToString converts the DQ_JointType to string.
     * @return A string that corresponds with the joint type.
     */
    std::string ToString() const {
        switch (joint_type_) {

        case REVOLUTE:
            return std::string("REVOLUTE");
        case PRISMATIC:
            return std::string("PRISMATIC");
        case SPHERICAL:
            return std::string("SPHERICAL");
        case CYLINDRICAL:
            return std::string("CYLINDRICAL");
        case PLANAR:
            return std::string("PLANAR");
        case SIX_DOF:
            return std::string("SIX_DOF");
        case HELICAL:
            return std::string("HELICAL");
        default:
            throw std::runtime_error("Invalid joint type");
        }
    }
};

}
