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
/**
 * @brief Encodes the actuation type of a robot joint.
 *
 * The available joint types follow Table 1 of Silva, Quiroz-Omaña, and
 * Adorno (2022), “Dynamics of Mobile Manipulators Using Dual Quaternion Algebra”.
 * The wrapper allows type-safe comparisons while still supporting conversion
 * from integral values used in matrix-based robot descriptions.
 *
 * @see DQ_SerialManipulator
 */
class DQ_JointType
{
public:
    /** @brief Enumeration of supported joint types. */
    enum JOINT_TYPE{
        REVOLUTE    = 0, /**< Revolute joint. */
        PRISMATIC,       /**< Prismatic joint. */
        SPHERICAL,       /**< Spherical joint. */
        CYLINDRICAL,     /**< Cylindrical joint. */
        PLANAR,          /**< Planar joint. */
        SIX_DOF,         /**< Six-degree-of-freedom joint. */
        HELICAL          /**< Helical joint. */
    };
    /**
     * @brief Converts the object to its underlying enumeration value.
     *
     * @return The stored joint-type enumeration.
     */
    constexpr operator JOINT_TYPE() const { return joint_type_; }
private:
    /** @brief Stored joint-type value. */
    JOINT_TYPE joint_type_;

public:
    /**
     * @brief Default constructor.
     */
    DQ_JointType() = default;

    /**
     * @brief Constructs the joint type from an enumeration value.
     *
     * @param joint_type Desired joint type.
     */
    DQ_JointType(const JOINT_TYPE& joint_type): joint_type_{joint_type}{};

    /**
     * @brief Constructs the joint type from an integer code.
     *
     * The accepted values are 0 for REVOLUTE, 1 for PRISMATIC, 2 for SPHERICAL,
     * 3 for CYLINDRICAL, 4 for PLANAR, 5 for SIX_DOF, and 6 for HELICAL.
     *
     * @param joint_type Integer code of the desired joint type.
     * @throws std::runtime_error If @p joint_type is outside the supported range.
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
     * @brief Converts the joint type to its uppercase string representation.
     *
     * @return String corresponding to the stored joint type.
     * @throws std::runtime_error If the stored value does not match a supported joint type.
     */
    std::string to_string() const {
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
