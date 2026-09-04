#pragma once
/**
(C) Copyright 2019-2022 DQ Robotics Developers

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

#include <dqrobotics/DQ.h>

using namespace Eigen;

namespace DQ_robotics
{
/**
 * @brief Abstract interface to quadratic-programming solvers used by DQ Robotics controllers.
 *
 * Concrete implementations solve optimization problems of the form
 * min 0.5*u'*H*u + f'*u subject to A*u <= b and Aeq*u = beq.
 *
 * @see DQ_QuadraticProgrammingController
 */
class DQ_QuadraticProgrammingSolver
{
protected:
    /**
     * @brief Default constructor for solver interfaces.
     */
    DQ_QuadraticProgrammingSolver() = default;
public:
    /**
     * @brief Virtual destructor.
     */
    virtual ~DQ_QuadraticProgrammingSolver() = default;

    /**
     * @brief Solves a quadratic program.
     *
     * Pure virtual interface contract implemented by concrete quadratic-programming solvers.
     *
     * @param H Symmetric matrix of the quadratic term.
     * @param f Vector of the linear term.
     * @param A Matrix of inequality constraints.
     * @param b Vector of inequality-constraint bounds.
     * @param Aeq Matrix of equality constraints.
     * @param beq Vector of equality-constraint bounds.
     * @return The optimal decision vector.
     */
    virtual VectorXd solve_quadratic_program(const MatrixXd& H, const VectorXd& f, const MatrixXd& A, const VectorXd& b, const MatrixXd& Aeq, const VectorXd& beq)=0;
};
}

