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
- Murilo M. Marinho (murilomarinho@ieee.org)
*/

#include<dqrobotics/robot_modeling/DQ_WholeBody.h>
#include<dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include<vector>

namespace DQ_robotics
{

void DQ_WholeBody::_check_to_ith_chain(const int &to_ith_chain) const
{
    if(to_ith_chain >= static_cast<int>(chain_.size()) || to_ith_chain < 0)
    {
        throw std::runtime_error(std::string("Tried to access chain index ") + std::to_string(to_ith_chain) + std::string(" which is unnavailable."));
    }
}

DQ_WholeBody::DQ_WholeBody(std::shared_ptr<DQ_Kinematics> robot)
{
    chain_.push_back(robot);
    dim_configuration_space_ = robot->get_dim_configuration_space();
}

DQ_Kinematics* DQ_WholeBody::get_chain(const int& to_ith_chain)
{
    _check_to_ith_chain(to_ith_chain);

    return chain_[static_cast<std::vector<DQ_Kinematics*>::size_type>(to_ith_chain)].get();
}

DQ_SerialManipulatorDH DQ_WholeBody::get_chain_as_serial_manipulator_dh(const int &to_ith_chain) const
{
    _check_to_ith_chain(to_ith_chain);

    try
    {
        return DQ_SerialManipulatorDH(*dynamic_cast<DQ_SerialManipulatorDH*>(chain_[static_cast<std::vector<DQ_Kinematics*>::size_type>(to_ith_chain)].get()));
    } catch (const std::bad_cast& e)
    {
        throw std::runtime_error("Index requested in get_chain_as_serial_manipulator_dh is not a DQ_SerialManipulatorDH" + std::string(e.what()));
    }
}

DQ_HolonomicBase DQ_WholeBody::get_chain_as_holonomic_base(const int &to_ith_chain) const
{
    _check_to_ith_chain(to_ith_chain);

    try
    {
        return DQ_HolonomicBase(*dynamic_cast<DQ_HolonomicBase*>(chain_[static_cast<std::vector<DQ_Kinematics*>::size_type>(to_ith_chain)].get()));
    } catch (const std::bad_cast& e)
    {
        throw std::runtime_error("Index requested in get_chain_as_holonomic_base is not a DQ_HolonomicBase" + std::string(e.what()));
    }
}

void DQ_WholeBody::add(std::shared_ptr<DQ_Kinematics> robot)
{
    dim_configuration_space_+= robot->get_dim_configuration_space();
    chain_.push_back(robot);
}

DQ DQ_WholeBody::fkm(const VectorXd& q) const
{
    return reference_frame_ * raw_fkm(q);
}

DQ DQ_WholeBody::fkm(const VectorXd& q, const int& to_chain) const
{
    return reference_frame_ * raw_fkm(q,to_chain);
}

DQ DQ_WholeBody::raw_fkm(const VectorXd &q) const
{
    return raw_fkm(q,chain_.size()-1);
}

DQ DQ_WholeBody::raw_fkm(const VectorXd &q, const int &to_ith_chain) const
{
    _check_to_ith_chain(to_ith_chain);

    DQ pose(1);

    int q_counter = 0;
    for(int i=0;i<to_ith_chain+1;i++)
    {
        const int current_robot_dim    = chain_[i]->get_dim_configuration_space();
        const VectorXd current_robot_q = q.segment(q_counter,current_robot_dim);
        pose = pose * chain_[i]->fkm(current_robot_q);
        q_counter += current_robot_dim;
    }

    return pose;
}

MatrixXd DQ_WholeBody::pose_jacobian(const VectorXd &q, const int &to_ith_chain) const
{
    _check_q_vec(q);
    _check_to_ith_chain(to_ith_chain);

    int n = chain_.size();
    DQ x_0_to_n = fkm(q,n-1);
    int q_counter = 0;

    //Not a good implementation but similar to MATLAB
    std::vector<MatrixXd> J_vector;
    for(int i=0;i<n;i++)
    {
        const int dim = chain_[i]->get_dim_configuration_space();

        const DQ x_0_to_iplus1 = fkm(q,i);
        const DQ x_iplus1_to_n = conj(x_0_to_iplus1)*x_0_to_n;

        const VectorXd q_iplus1 = q.segment(q_counter,dim);
        q_counter += dim;

        if(i==0)
            J_vector.push_back(haminus8(x_iplus1_to_n)*chain_[i]->pose_jacobian(q_iplus1,dim-1));
        else
            J_vector.push_back(hamiplus8(fkm(q,i-1))*haminus8(x_iplus1_to_n)*chain_[i]->pose_jacobian(q_iplus1,dim-1));
    }

    MatrixXd J_pose(8,q_counter);
    int col_counter = 0;
    for(int i=0;i<n;i++)
    {
        int q_counter = chain_[i]->get_dim_configuration_space();
        MatrixXd current_J = J_vector[i];
        for(int j=0;j<q_counter;j++)
        {
            J_pose.col(col_counter)=current_J.col(j);
            col_counter+=1;
        }
    }
    return J_pose;
}

MatrixXd DQ_WholeBody::pose_jacobian(const VectorXd &q) const
{
    return pose_jacobian(q, chain_.size()-1); //The behavior for fkm(q, to_link) is weird in 19.10
}

void DQ_WholeBody::set_effector(const DQ &effector)
{
    try
    {
        std::dynamic_pointer_cast<DQ_SerialManipulator>(chain_[chain_.size()-1])->set_effector(effector);
    }
    catch (const std::bad_cast& e)
    {
        throw std::runtime_error("The last element of the chain needs to be a DQ_SerialManipulator to use set_effector in a DQ_WholeBody " + std::string(e.what()));
    }
}

/**
 * @brief returns the Jacobian derivative 'J_dot' that satisfies
          vec8(pose_dot_dot) = J_dot * q_dot + J*q_dot_dot, where pose = fkm(), 'pose_dot' is the time
          derivative of the pose and 'q_dot' represents the robot configuration velocities.
 * @param q The VectorXd representing the robot configurations.
 * @param q_dot The VectorXd representing the robot configuration velocities.
 * @param to_ith_link The 'to_ith_link' link which we want to compute the Jacobian derivative.
 * @return a MatrixXd representing the desired Jacobian derivative.
 */
MatrixXd DQ_WholeBody::pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot, const int &to_ith_link) const
{
    throw std::runtime_error("pose_jacobian_derivative is not implemented yet.");
}


/**
 * @brief returns the Jacobian derivative 'J_dot' that satisfies
          vec8(pose_dot_dot) = J_dot * q_dot + J*q_dot_dot, where pose = fkm(), 'pose_dot' is the time
          derivative of the pose and 'q_dot' represents the robot configuration velocities.
 * @param q The VectorXd representing the robot configurations.
 * @param q_dot The VectorXd representing the robot configuration velocities.
 * @return a MatrixXd representing the desired Jacobian derivative.
 */
MatrixXd DQ_WholeBody::pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot) const
{
    return pose_jacobian_derivative(q,q_dot,  get_dim_configuration_space()-1);
}

}
