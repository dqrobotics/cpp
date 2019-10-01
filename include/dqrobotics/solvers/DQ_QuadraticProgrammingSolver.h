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

#ifndef DQ_SOLVERS_DQ_QUADRATICPROGRAMMINGSOLVER_H
#define DQ_SOLVERS_DQ_QUADRATICPROGRAMMINGSOLVER_H

#include <dqrobotics/DQ.h>

using namespace Eigen;

namespace DQ_robotics
{
class DQ_QuadraticProgrammingSolver
{
public:
    //Remove default constructor
    DQ_QuadraticProgrammingSolver() = delete;

    virtual VectorXd solve_quadratic_program(const MatrixXd& H, const MatrixXd& f, const MatrixXd A, const MatrixXd& b, const MatrixXd& Aeq, const MatrixXd& beq)=0;
};
}

#endif
