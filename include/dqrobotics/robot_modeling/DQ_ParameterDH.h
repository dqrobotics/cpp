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

#include <unordered_map>
#include <iostream>
#pragma once

namespace DQ_robotics
{
/**
 * @brief Selects a Denavit-Hartenberg parameter by name.
 *
 * DQ_ParameterDH is a small wrapper used by the C++ API to refer to the
 * rows of a DH or modified-DH matrix. It also accepts string-based
 * construction to keep the interface aligned with other DQ Robotics bindings.
 *
 * @see DQ_SerialManipulatorDH, DQ_SerialManipulatorMDH
 */
class DQ_ParameterDH
{
public:
    /** @brief Enumeration of supported Denavit-Hartenberg parameters. */
    enum PARAMETER{
        THETA, /**< Joint-angle parameter. */
        D,     /**< Link offset parameter. */
        A,     /**< Link-length parameter. */
        ALPHA  /**< Link-twist parameter. */
    };
private:
    /** @brief Stored Denavit-Hartenberg parameter. */
    PARAMETER parameter_;
    /** @brief Mapping from uppercase strings to supported Denavit-Hartenberg parameters. */
    const std::unordered_map<std::string, PARAMETER>
        map_ = {{"THETA", THETA},
                {"D"    ,     D},
                {"A"    ,     A},
                {"ALPHA", ALPHA},
                };

    /**
     * @brief Sets the stored parameter from a string.
     *
     * @param parameter Name of the desired parameter.
     * @throws std::runtime_error If @p parameter is not one of THETA, D, A, or ALPHA.
     */
    void _set_parameter(const std::string& parameter)
    {
        try {
            parameter_ = map_.at(parameter);
        } catch (...) {
            throw std::runtime_error("The parameter \""+ parameter+ "\" is not supported. Use THETA, D, A, or ALPHA");
        }
    }
public:
    /**
     * @brief Default constructor.
     */
    DQ_ParameterDH() = default;

    /**
     * @brief Constructs the selector from an enumeration value.
     *
     * @param parameter Desired DH parameter.
     */
    DQ_ParameterDH(const PARAMETER& parameter): parameter_{parameter}{};

    /**
     * @brief Converts the object to its underlying enumeration value.
     *
     * @return The stored parameter enumeration.
     */
    constexpr operator PARAMETER() const { return parameter_; }

    /**
     * @brief Constructs the selector from a string.
     *
     * This constructor keeps the C++ interface compatible with string-based
     * parameter selection used in other DQ Robotics language bindings.
     *
     * @param parameter Desired DH parameter as a string.
     * @throws std::runtime_error If @p parameter is not one of THETA, D, A, or ALPHA.
     */
    DQ_ParameterDH(const std::string& parameter){
        _set_parameter(parameter);
    }


    /**
     * @brief Constructs the selector from a C string.
     *
     * This constructor keeps the C++ interface compatible with string-based
     * parameter selection used in other DQ Robotics language bindings.
     *
     * @param parameter_c Desired DH parameter as a C string.
     * @throws std::runtime_error If @p parameter_c is not one of THETA, D, A, or ALPHA.
     */
    DQ_ParameterDH(const char* parameter_c){
        _set_parameter(parameter_c);
    }

};
}


