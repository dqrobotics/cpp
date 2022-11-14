/**
(C) Copyright 2020-2022 DQ Robotics Developers

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

#include<dqrobotics/robot_modeling/DQ_SerialWholeBody.h>
#include<vector>

namespace DQ_robotics
{

void DQ_SerialWholeBody::_check_to_ith_chain(const int &to_ith_chain) const
{
    if(to_ith_chain >= static_cast<int>(chain_.size()) || to_ith_chain < 0)
    {
        throw std::runtime_error(std::string("Tried to access chain index ") + std::to_string(to_ith_chain) + std::string(" which is unnavailable."));
    }
}

void DQ_SerialWholeBody::_check_to_jth_link_of_ith_chain(const int &to_ith_chain, const int &to_jth_link) const
{
    _check_to_ith_chain(to_ith_chain);
    if( (to_jth_link >= static_cast<int>(chain_[to_ith_chain]->get_dim_configuration_space())) || to_jth_link < 0)
    {
        throw std::runtime_error(
                    std::string("Tried to access link index ")
                    + std::to_string(to_jth_link)
                    + std::string(" of chain index ")
                    + std::to_string(to_ith_chain)
                    + std::string(" which is unnavailable.")
                    );
    }
}

DQ_SerialWholeBody::DQ_SerialWholeBody(std::shared_ptr<DQ_Kinematics> robot, const std::string type)
{
    chain_.push_back(robot);
    dim_configuration_space_ = robot->get_dim_configuration_space();
    if(type == std::string("standard"))
    {
        //Nothing to do
    }
    else if(type == std::string("reversed"))
    {
        throw std::runtime_error(std::string("Reversed type DQ_SerialWholeBody is not implemented yet"));
    }
    else
    {
        throw std::runtime_error(std::string("Invalid type: ") + type);
    }
}

DQ_Kinematics* DQ_SerialWholeBody::get_chain(const int& to_ith_chain)
{
    _check_to_ith_chain(to_ith_chain);

    return chain_[static_cast<std::vector<DQ_Kinematics*>::size_type>(to_ith_chain)].get();
}

DQ_SerialManipulatorDH DQ_SerialWholeBody::get_chain_as_serial_manipulator_dh(const int &to_ith_chain) const
{
    _check_to_ith_chain(to_ith_chain);

    try
    {
        return DQ_SerialManipulatorDH(*dynamic_cast<DQ_SerialManipulatorDH*>(chain_[static_cast<std::vector<DQ_Kinematics*>::size_type>(to_ith_chain)].get()));
    } catch (const std::bad_cast& e)
    {
        throw std::runtime_error("Index requested in get_chain_as_serial_manipulator is not a SerialManipulator" + std::string(e.what()));
    }
}

DQ_HolonomicBase DQ_SerialWholeBody::get_chain_as_holonomic_base(const int &to_ith_chain) const
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

void DQ_SerialWholeBody::add(std::shared_ptr<DQ_Kinematics> robot)
{
    dim_configuration_space_+= robot->get_dim_configuration_space();
    chain_.push_back(robot);
}

DQ DQ_SerialWholeBody::fkm(const VectorXd& q) const
{
    return reference_frame_ * raw_fkm(q);
}

DQ DQ_SerialWholeBody::fkm(const VectorXd& q, const int& to_ith_link) const
{
    return reference_frame_ * raw_fkm(q,to_ith_link);
}

std::tuple<int,int> DQ_SerialWholeBody::get_chain_and_link_from_index(const int &to_ith_link) const
{
    int ith_chain = 0;
    int jth_link = 0;

    int n = to_ith_link;
    for(size_t ith=0;ith<chain_.size();ith++)
    {
        if( (n - chain_[ith]->get_dim_configuration_space()) >= 0)
        {
            ith_chain++;
            n = n - chain_[ith]->get_dim_configuration_space();
        }
        else
        {
            jth_link = n;
            return std::make_tuple(ith_chain, jth_link);
        }
    }
    throw std::runtime_error("Unable to get_chain_and_link_from_index.");
}

DQ DQ_SerialWholeBody::raw_fkm(const VectorXd &q) const
{
    return raw_fkm_by_chain(q, chain_.size()-1, chain_[chain_.size()-1]->get_dim_configuration_space()-1);
}

DQ DQ_SerialWholeBody::raw_fkm(const VectorXd &q, const int &to_ith_link) const
{
    int to_ith_chain;
    int to_jth_link;
    std::tie(to_ith_chain,to_jth_link) = get_chain_and_link_from_index(to_ith_link);

    return raw_fkm_by_chain(q, to_ith_chain, to_jth_link);
}

DQ DQ_SerialWholeBody::raw_fkm_by_chain(const VectorXd &q, const int &to_ith_chain, const int &to_jth_link) const
{
    _check_q_vec(q);
    _check_to_ith_chain(to_ith_chain);

    DQ pose(1);

    int q_counter = 0;
    int current_robot_dim;
    VectorXd current_robot_q;
    //Loop until the second-to-last element until to_ith_chain
    for(int i=0;i<to_ith_chain;i++)
    {
        current_robot_dim = chain_[i]->get_dim_configuration_space();
        current_robot_q   = q.segment(q_counter,current_robot_dim);
        pose = pose * chain_[i]->fkm(current_robot_q);
        q_counter += current_robot_dim;
    }
    //The last element in the chain can be partial
    current_robot_dim  = chain_[to_ith_chain]->get_dim_configuration_space();
    current_robot_q    = q.segment(q_counter,current_robot_dim);
    pose = pose * chain_[to_ith_chain]->fkm(current_robot_q,to_jth_link);

    return pose;
}

DQ DQ_SerialWholeBody::raw_fkm_by_chain(const VectorXd &q, const int &to_ith_chain) const
{
    _check_q_vec(q);
    _check_to_ith_chain(to_ith_chain);

    return raw_fkm_by_chain(q,to_ith_chain,chain_[to_ith_chain]->get_dim_configuration_space()-1);
}

MatrixXd DQ_SerialWholeBody::raw_pose_jacobian_by_chain(const VectorXd &q, const int &to_ith_chain, const int &to_jth_link) const
{
    _check_q_vec(q);
    _check_to_jth_link_of_ith_chain(to_ith_chain, to_jth_link);

    int n = to_ith_chain+1; //Size of the partial or total chain
    DQ x_0_to_n = raw_fkm_by_chain(q,to_ith_chain,to_jth_link);
    int q_counter = 0;

    //Not the best implementation but similar to MATLAB
    std::vector<MatrixXd> J_vector;
    for(int i=0;i<n;i++)
    {
        int dim = chain_[i]->get_dim_configuration_space();
        // Addressing the to_jth_link

        const DQ x_0_to_iplus1 = raw_fkm_by_chain(q,i);
        const DQ x_iplus1_to_n = conj(x_0_to_iplus1)*x_0_to_n;

        const VectorXd q_iplus1 = q.segment(q_counter,dim);
        q_counter += dim;

        if(i==0)
        {
            J_vector.push_back(haminus8(x_iplus1_to_n)*chain_[i]->pose_jacobian(q_iplus1,dim-1));
        }
        else
        {
            if(i==n-1) //To address the last index in the chain, or partial chains
                J_vector.push_back(hamiplus8(raw_fkm_by_chain(q,i-1))*haminus8(x_iplus1_to_n)*chain_[i]->pose_jacobian(q_iplus1,to_jth_link));
            else
                J_vector.push_back(hamiplus8(raw_fkm_by_chain(q,i-1))*haminus8(x_iplus1_to_n)*chain_[i]->pose_jacobian(q_iplus1,dim-1));
        }
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

MatrixXd DQ_SerialWholeBody::pose_jacobian(const VectorXd &q, const int &to_ith_link) const
{
    int to_ith_chain;
    int to_jth_link;
    std::tie(to_ith_chain,to_jth_link) = get_chain_and_link_from_index(to_ith_link);

    return raw_pose_jacobian_by_chain(q,to_ith_chain,to_jth_link);
}

MatrixXd DQ_SerialWholeBody::pose_jacobian(const VectorXd &q) const
{
    return pose_jacobian(q, get_dim_configuration_space()-1);
}

//To be implemented.
MatrixXd DQ_SerialWholeBody::raw_pose_jacobian_derivative_by_chain(const VectorXd &configurations, const VectorXd &velocity_configurations, const int &to_ith_chain, const int &to_jth_link) const
{
    throw std::runtime_error(std::string("pose_jacobian_derivative_by_chain is not implemented yet."));
    return MatrixXd::Zero(1,1);
}

/**
 * @brief returns the Jacobian derivative 'J_dot' that satisfies
          vec8(pose_dot_dot) = J_dot * q_dot + J*q_dot_dot, where pose = fkm(), 'pose_dot' is the time
          derivative of the pose and 'q_dot' represents the robot velocity configurations.
 * @param configurations The VectorXd representing the robot configurations.
 * @param velocity_configurations The VectorXd representing the robot velocity configurations.
 * @param to_ith_link The 'to_ith_link' link which we want to compute the Jacobian derivative.
 * @return a MatrixXd representing the desired Jacobian derivative.
 */
MatrixXd DQ_SerialWholeBody::pose_jacobian_derivative(const VectorXd &configurations, const VectorXd &velocity_configurations, const int &to_ith_link) const
{
    int to_ith_chain;
    int to_jth_link;

    /// Aliases
    const VectorXd& q = configurations;
    const VectorXd& q_dot = velocity_configurations;

    std::tie(to_ith_chain,to_jth_link) = get_chain_and_link_from_index(to_ith_link);
    return raw_pose_jacobian_derivative_by_chain(q, q_dot, to_ith_chain,to_ith_link);
}


/**
 * @brief returns the Jacobian derivative 'J_dot' that satisfies
          vec8(pose_dot_dot) = J_dot * q_dot + J*q_dot_dot, where pose = fkm(), 'pose_dot' is the time
          derivative of the pose and 'q_dot' represents the robot velocity configurations.
 * @param configurations The VectorXd representing the robot configurations.
 * @param velocity_configurations The VectorXd representing the robot velocity configurations.
 * @return a MatrixXd representing the desired Jacobian derivative.
 */
MatrixXd DQ_SerialWholeBody::pose_jacobian_derivative(const VectorXd &configurations, const VectorXd &velocity_configurations) const
{
    /// Aliases
    const VectorXd& q = configurations;
    const VectorXd& q_dot = velocity_configurations;
    return pose_jacobian_derivative(q,q_dot,  get_dim_configuration_space()-1);
}

void DQ_SerialWholeBody::set_effector(const DQ &effector)
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

}
