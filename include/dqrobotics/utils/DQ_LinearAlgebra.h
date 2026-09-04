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

#include <Eigen/Dense>
#include <tuple>

using namespace Eigen;

namespace DQ_robotics
{

/**
 * @brief Computes the rank of a matrix using singular value decomposition.
 *
 * The tolerance matches the MATLAB-inspired rule used by this library,
 * namely `max(rows, cols) * sigma_max * eps`, where `sigma_max` is the
 * largest singular value of the matrix.
 *
 * @param matrix The input matrix.
 * @return The number of singular values greater than the default tolerance.
 * @see pinv, svd
 */
int rank(const MatrixXd& matrix);

/**
 * @brief Computes the Moore-Penrose pseudoinverse of a matrix.
 *
 * The pseudoinverse is obtained from a full singular value decomposition.
 * Singular values smaller than the MATLAB-style tolerance
 * `max(rows, cols) * sigma_max * eps` are treated as zero.
 *
 * @param matrix The input matrix.
 * @return The pseudoinverse of the input matrix.
 * @see rank, svd
 */
MatrixXd pinv(const MatrixXd& matrix);

/**
 * @brief Computes the singular value decomposition of a matrix.
 *
 * The returned tuple is ordered as `(U, S, V)` so the original matrix can be
 * reconstructed as `matrix = U * S * V.adjoint()`.
 *
 * @param matrix The input matrix.
 * @return A tuple containing the left singular vectors, the diagonal matrix of
 * singular values, and the right singular vectors.
 * @see pinv, rank
 */
std::tuple<MatrixXd,MatrixXd,MatrixXd> svd(const MatrixXd& matrix);

}

