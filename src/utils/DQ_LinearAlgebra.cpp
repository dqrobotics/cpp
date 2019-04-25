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

#include<dqrobotics/utils/DQ_LinearAlgebra.h>

namespace DQ_robotics
{

/**
 * @brief pinv Calculates the pseudo inverse of the input @p matrix using Singular Value Decomposition with
 * a given tolerance for small singular values.
 * http://www.mathworks.com/help/matlab/ref/pinv.html
 * @param matrix the input matrix
 * @return the pseudo-inverse of @p matrix such that pinv(matrix)*matrix is as close as possible to the identity matrix.
 */
MatrixXd pinv(const MatrixXd& matrix)
{
    int num_rows = matrix.rows();
    int num_cols = matrix.cols();

    MatrixXd pseudo_inverse(num_cols,num_rows);
    JacobiSVD<MatrixXd> svd(num_cols,num_rows);
    VectorXd singular_values;
    MatrixXd svd_sigma_inverted(num_cols,num_rows);
    svd_sigma_inverted = MatrixXd::Zero(num_cols,num_rows);

    svd.compute(matrix, ComputeFullU | ComputeFullV);
    singular_values = svd.singularValues();

    //Tolerance Calculation
    double eps =  std::numeric_limits<double>::epsilon();
    int max =  (num_rows > num_cols) ? num_rows : num_cols;
    double norm = singular_values(0); //Matlab uses the 2-NORM, which is the largest singular value. Meyer p.281
    double tol = max*norm*eps;

    for(int i=0;i<singular_values.size();i++)
    {
        if(singular_values(i) > tol)
            svd_sigma_inverted(i,i) = 1/(singular_values(i));
        else
            svd_sigma_inverted(i,i) = 0;
    }

    pseudo_inverse = svd.matrixV() * (svd_sigma_inverted * svd.matrixU().adjoint());

    return pseudo_inverse;
}

}
