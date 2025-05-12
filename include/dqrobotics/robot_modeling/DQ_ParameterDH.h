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
class DQ_ParameterDH
{
public:
    enum PARAMETER{
        THETA,
        D,
        A,
        ALPHA
    };
private:
    PARAMETER parameter_;
    const std::unordered_map<std::string, PARAMETER>
        map_ = {{"THETA", THETA},
                {"D"    ,     D},
                {"A"    ,     A},
                {"ALPHA", ALPHA},
                };

    /**
     * @brief _get_parameter sets the parameter member using a string as argument.
     * @param parameter The desired parameter to be set. Example: "THETA", "D", "A", or "ALPHA".
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
    DQ_ParameterDH() = default;
    DQ_ParameterDH(const PARAMETER& parameter): parameter_{parameter}{};

    // This definition enables switch cases and comparisons.
    constexpr operator PARAMETER() const { return parameter_; }

    // This constructor allows string parameters. This is done to keep the
    // language compatibility between Matlab and Python/C++, as discussed in
    //https://github.com/dqrobotics/cpp/pull/69
    DQ_ParameterDH(const std::string& parameter){
        _set_parameter(parameter);
    }

    // This constructor allows char parameters. This is done to keep the
    // language compatibility between Matlab and Python/C++, as discussed in
    //https://github.com/dqrobotics/cpp/pull/69
    DQ_ParameterDH(const char* parameter_c){
        _set_parameter(parameter_c);
    }

};
}


