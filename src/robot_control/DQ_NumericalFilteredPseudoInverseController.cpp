/**
(C) Copyright 2022 DQ Robotics Developers

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

#include <dqrobotics/robot_control/DQ_NumericalFilteredPseudoInverseController.h>
#include <dqrobotics/utils/DQ_LinearAlgebra.h>

namespace DQ_robotics
{


DQ_NumericalFilteredPseudoinverseController::DQ_NumericalFilteredPseudoinverseController(DQ_Kinematics *robot):
    DQ_PseudoinverseController (robot),
    epsilon_(0),
    lambda_max_(0),
    last_jacobian_rank_(-1)
{

}

DQ_NumericalFilteredPseudoinverseController::DQ_NumericalFilteredPseudoinverseController(const std::shared_ptr<DQ_Kinematics> &robot):
    DQ_PseudoinverseController (robot),
    epsilon_(0),
    lambda_max_(0),
    last_jacobian_rank_(-1)
{

}

VectorXd DQ_NumericalFilteredPseudoinverseController::compute_setpoint_control_signal(const VectorXd& q, const VectorXd& task_reference)
{
    return DQ_NumericalFilteredPseudoinverseController::compute_tracking_control_signal(q, task_reference, VectorXd::Zero(task_reference.size()));
}

VectorXd DQ_NumericalFilteredPseudoinverseController::compute_tracking_control_signal(const VectorXd &q, const VectorXd &task_reference, const VectorXd &feed_forward)
{
    //Trivial lambda_max_ and epsilon_ mean it can be calculated using the DQ_PseudoinverseController
    if(lambda_max_==0 || epsilon_==0)
        return DQ_PseudoinverseController::compute_tracking_control_signal(q,task_reference,feed_forward);

    if(is_set())
    {
        const VectorXd& task_variable = get_task_variable(q);
        const VectorXd& task_error = task_variable-task_reference;

        const MatrixXd& J = get_jacobian(q);
        const int& jacobian_rank = rank(J);
        last_jacobian_rank_ = jacobian_rank;
        const int max_rank = std::min(J.rows(),J.cols());
        if(jacobian_rank==max_rank)
        {
            //When the matrix is full rank, it can be calculated using DQ_PseudoinverseController
            return DQ_PseudoinverseController::compute_tracking_control_signal(q,task_reference,feed_forward);
        }

        MatrixXd U, S, V;
        std::tie(U,S,V) = svd(J);
        last_jacobian_svd_ = {U,S,V};
        MatrixXd total_filtered_damping;
        for(auto i=0;i<jacobian_rank;i++)
        {
            if(S(i,i)<epsilon_)
            {
                //My interpretation of the paper is that the level of damping will must vary
                //between singular values
                double lambda_squared_i = (1.0 - std::pow(S(i,i)/epsilon_,2))*std::pow(lambda_max_,2);//Eq. (15)
                MatrixXd filtered_damping_i = lambda_squared_i*(U.col(i)*U.col(i).transpose());
                if(total_filtered_damping.size()==0)
                    total_filtered_damping = filtered_damping_i;
                else
                    total_filtered_damping += filtered_damping_i;

            }
        }
        last_filtered_damping_ = total_filtered_damping;
        if(total_filtered_damping.size()==0)
        {
            //When the filter is zero, the controller reduces itself to the pseudoinverse controller
            return DQ_PseudoinverseController::compute_tracking_control_signal(q,task_reference,feed_forward);
        }


        VectorXd u = J.transpose()*
                (J*J.transpose()
                 + damping_*damping_*MatrixXd::Identity(q.size(), q.size())
                 + total_filtered_damping
                 ).inverse()*(-gain_*task_error + feed_forward);

        verify_stability(task_error);

        last_control_signal_ = u;
        last_error_signal_   = task_error;
        return u;
    }
    else
    {
        throw std::runtime_error("Tried computing the control signal of an unset controller.");
    }
}

void DQ_NumericalFilteredPseudoinverseController::set_maximum_numerical_filtered_damping(const double &numerical_filtered_damping)
{
    lambda_max_ = numerical_filtered_damping;
}

void DQ_NumericalFilteredPseudoinverseController::set_singular_region_size(const double &singular_region_size)
{
    if(singular_region_size < 0)
        throw std::range_error(std::string(__func__)
                               +"::Singular region size must be >= 0 and not "
                               +std::to_string(singular_region_size)
                               +".");
    epsilon_ = singular_region_size;
}

double DQ_NumericalFilteredPseudoinverseController::get_maximum_numerical_filtered_damping() const
{
    return lambda_max_;
}

double DQ_NumericalFilteredPseudoinverseController::get_singular_region_size() const
{
    return epsilon_;
}

MatrixXd DQ_NumericalFilteredPseudoinverseController::get_last_filtered_damping() const
{
    return last_filtered_damping_;
}

int DQ_NumericalFilteredPseudoinverseController::get_last_jacobian_rank() const
{
    return last_jacobian_rank_;
}

std::tuple<MatrixXd, MatrixXd, MatrixXd> DQ_NumericalFilteredPseudoinverseController::get_last_jacobian_svd() const
{
    return last_jacobian_svd_;
}

}

