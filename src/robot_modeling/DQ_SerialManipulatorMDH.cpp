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
1. Murilo M. Marinho (murilomarinho@ieee.org)
    - Responsible for the original implementation.

2. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
    - Added methods to get and set the DH parameters.
    - Added the get_supported_joint_types() method.
*/

#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>

namespace DQ_robotics
{

/**
 * @brief Constructor of the DQ_SerialManipulatorMDH class.
 * @param mdh_matrix The matrix 5 x number_of_joints that represent the MDH parameters of the robot.
 * 
 *                  Example: Consider the Stanford manipulator. Using the MDH parameters, we have: 
 *                  (See Table 2.1 from Foundations of Robotics, Tsuneo Yoshikawa)
 * 
 *                  Matrix<double, 5, 6> robot_mdh;
 *                  robot_mdh << 0, 0, 0, 0, 0, 0,                        // theta
 *                               0,d2,d3,0,0,0,                           // d
 *                               0, 0, 0, 0, 0, 0,                        // a
 *                               0, -M_PI_2, M_PI_2,  0, -M_PI_2, M_PI_2, // alpha
 *                               0,0,1,0,0,0;  // Type of joints. The joints are rotational, except the third joint, which is prismatic.
 *                  DQ_SerialManipulatorMDH StandfordManipulator(robot_mdh); 
 * 
 *                  // The MDH parameters of this robot does not take
 *                  // into account the constant translation of the end effector.    
 *                  StandfordManipulator.set_effector(1+E_*0.5*k_*d6); 
 *
 */
DQ_SerialManipulatorMDH::DQ_SerialManipulatorMDH(const MatrixXd& mdh_matrix):
    DQ_SerialManipulator(mdh_matrix.cols())
{
    if(mdh_matrix.rows() != 5)
    {
        throw(std::range_error("Bad DQ_SerialManipulatorDH(mdh_matrix) call: mdh_matrix should be 5xn"));
    }
    mdh_matrix_ = mdh_matrix;
    set_joint_types(mdh_matrix.row(4));
}

/**
 * @brief This protected method computes the unit dual quaternion for a given link's Extended MDH parameters.
 * @param q The joint value.
 * @param ith The link number.
 * @returns The unit dual quaternion that correspond for a given link's Extended MDH parameters. 
 * 
 *              Example: DQ x = _mdh2dq(q, ith);
 * 
 */
DQ DQ_SerialManipulatorMDH::_mdh2dq(const double &q, const int &ith) const
{
    double half_theta = mdh_matrix_(0,ith)/2.0;
    double d = mdh_matrix_(1,ith);
    const double &a = mdh_matrix_(2,ith);
    const double half_alpha = mdh_matrix_(3,ith)/2.0;
    const int joint_type = int(mdh_matrix_(4,ith));

    // Add the effect of the joint value
    if(joint_type == JOINT_ROTATIONAL)
    {
        half_theta = half_theta + (q/2.0);
    }
    else
    {
        d = d + q;
    }

    // Pre-calculate cosines and sines
    const double sine_of_half_theta = sin(half_theta);
    const double cosine_of_half_theta = cos(half_theta);
    const double sine_of_half_alpha = sin(half_alpha);
    const double cosine_of_half_alpha = cos(half_alpha);

    // Return the optimized standard mdh2dq calculation
    return DQ(
                cosine_of_half_alpha*cosine_of_half_theta,
                sine_of_half_alpha*cosine_of_half_theta,
                -sine_of_half_alpha*sine_of_half_theta, //MDH
                cosine_of_half_alpha*sine_of_half_theta,

                -(a*sine_of_half_alpha*cosine_of_half_theta) /2.0 
                - (d*cosine_of_half_alpha*sine_of_half_theta)/2.0,

                (a*cosine_of_half_alpha*cosine_of_half_theta)/2.0
                 - (d*sine_of_half_alpha*sine_of_half_theta  )/2.0,

                -(a*cosine_of_half_alpha*sine_of_half_theta)  /2.0
                 - (d*sine_of_half_alpha*cosine_of_half_theta)/2.0, //MDH

                (d*cosine_of_half_alpha*cosine_of_half_theta)/2.0
                 - (a*sine_of_half_alpha*sine_of_half_theta  )/2.0
                );
}

/**
 * @brief DQ_SerialManipulatorMDH::get_parameters returns a vector containing the MDH parameters.
 * @param parameter_type Parameter type, which which corresponds to THETA, D, A, or ALPHA.
 * @return A vector containing the desired MDH parameters.
 */
VectorXd DQ_SerialManipulatorMDH::get_parameters(const DQ_ParameterDH &parameter_type) const
{
    switch (parameter_type) {
    case DQ_ParameterDH::THETA:
        return  mdh_matrix_.row(0);
    case DQ_ParameterDH::D:
        return  mdh_matrix_.row(1);
    case DQ_ParameterDH::A:
        return  mdh_matrix_.row(2);
    case DQ_ParameterDH::ALPHA:
        return  mdh_matrix_.row(3);
    default:
        throw std::runtime_error("Wrong type of parameter");
    }
}

/**
 * @brief DQ_SerialManipulatorMDH::get_parameter returns the MDH parameter of the ith joint.
 * @param parameter_type Parameter type, which which corresponds to THETA, D, A, or ALPHA.
 * @param to_ith_link The joint number.
 * @return The desired MDH parameter.
 */
double DQ_SerialManipulatorMDH::get_parameter(const DQ_ParameterDH &parameter_type,
                                             const int &to_ith_link) const
{
    _check_to_ith_link(to_ith_link);
    switch (parameter_type) {
    case DQ_ParameterDH::THETA:
        return  mdh_matrix_(0, to_ith_link);
    case DQ_ParameterDH::D:
        return  mdh_matrix_(1, to_ith_link);
    case DQ_ParameterDH::A:
        return  mdh_matrix_(2, to_ith_link);
    case DQ_ParameterDH::ALPHA:
        return  mdh_matrix_(3, to_ith_link);
    default:
        throw std::runtime_error("Wrong type of parameter");
    }
}

/**
 * @brief DQ_SerialManipulatorMDH::set_parameters sets the MDH parameters.
 * @param parameter_type Parameter type, which which corresponds to THETA, D, A, or ALPHA.
 * @param vector_parameters A vector containing the new parameters.
 */
void DQ_SerialManipulatorMDH::set_parameters(const DQ_ParameterDH &parameter_type,
                                            const VectorXd &vector_parameters)
{
    _check_q_vec(vector_parameters);
    switch (parameter_type) {
    case DQ_ParameterDH::THETA:
        mdh_matrix_.row(0) = vector_parameters;
        break;
    case DQ_ParameterDH::D:
        mdh_matrix_.row(1) =  vector_parameters;
        break;
    case DQ_ParameterDH::A:
        mdh_matrix_.row(2) =  vector_parameters;
        break;
    case DQ_ParameterDH::ALPHA:
        mdh_matrix_.row(3) =  vector_parameters;
        break;
    }
}

/**
 * @brief DQ_SerialManipulatorMDH::set_parameter sets the MDH parameter of the ith joint.
 * @param parameter_type Parameter type, which which corresponds to THETA, D, A, or ALPHA.
 * @param to_ith_link The joint number.
 * @param parameter The new parameter.
 */
void DQ_SerialManipulatorMDH::set_parameter(const DQ_ParameterDH &parameter_type,
                                           const int &to_ith_link, const double &parameter)
{
    _check_to_ith_link(to_ith_link);
    switch (parameter_type) {
    case DQ_ParameterDH::THETA:
        mdh_matrix_(0, to_ith_link) = parameter;
        break;
    case DQ_ParameterDH::D:
        mdh_matrix_(1, to_ith_link) = parameter;
        break;
    case DQ_ParameterDH::A:
        mdh_matrix_(2, to_ith_link) = parameter;
        break;
    case DQ_ParameterDH::ALPHA:
        mdh_matrix_(3, to_ith_link) = parameter;
        break;
    }
}

/**
 * @brief DQ_SerialManipulatorMDH::get_supported_joint_types gets the supported joint types.
 * @return A vector containing the supported joint types.
 */
std::vector<DQ_JointType> DQ_SerialManipulatorMDH::get_supported_joint_types() const
{
    return {DQ_JointType::REVOLUTE, DQ_JointType::PRISMATIC};
}


/**
 * @brief This protected method computes the dual quaternion related with the time derivative of the
 *        unit dual quaternion pose using the MDH convention. 
 *        (See. eq (2.27) of 'Two-arm Manipulation: From Manipulators to Enhanced Human-Robot Collaboration', Bruno Vilhena Adorno).
 *
 * @param ith The link number.
 * @returns The dual quaternion related with the time derivative of the unit dual quaternion pose using the MDH convention. 
 * 
 *              Example: DQ w = _get_w(ith);
 * 
 */
DQ DQ_SerialManipulatorMDH::_get_w(const int &ith) const
{
    const int joint_type = int(mdh_matrix_(4,ith));
    const double alpha = mdh_matrix_(3,ith);
    const double &a = mdh_matrix_(2,ith);
    if(joint_type == JOINT_ROTATIONAL)
        return -j_*sin(alpha)+ k_*cos(alpha)- E_*a*(j_*cos(alpha) + k_*sin(alpha));
    else
        return E_*(cos(alpha)*k_ - sin(alpha)*j_);
}

/**
 * @brief This method returns the first row of the Matrix mdh_matrix_, which
 *        corresponds to the parameter 'theta' in the MDH convention.
 *  
 * @returns The first row of the Matrix mdh_matrix_, which  corresponds
 *          to the parameter 'theta' in the MDH convention.   
 * 
 */
VectorXd  DQ_SerialManipulatorMDH::get_thetas() const
{
    return mdh_matrix_.row(0);
}

/**
 * @brief This method returns the second row of the Matrix mdh_matrix_, which
 *        corresponds to the parameter 'd' in the MDH convention.
 *  
 * @returns The second row of the Matrix mdh_matrix_, which corresponds
 *          to the parameter 'd' in the MDH convention.   
 * 
 */
VectorXd  DQ_SerialManipulatorMDH::get_ds() const
{
    return mdh_matrix_.row(1);
}

/**
 * @brief This method returns the third row of the Matrix mdh_matrix_, which
 *        corresponds to the parameter 'a' in the MDH convention.
 *  
 * @returns The third row of the Matrix mdh_matrix_, which corresponds to
 *          the parameter 'a' in the MDH convention.  
 * 
 */
VectorXd  DQ_SerialManipulatorMDH::get_as() const
{
    return mdh_matrix_.row(2);
}

/**
 * @brief This method returns the fourth row of the Matrix mdh_matrix_, which
 *        corresponds to the parameter 'alpha' in the MDH convention.
 * 
 * @returns The fourth row of the Matrix mdh_matrix_, which corresponds to
 *          the parameter 'alpha' in the MDH convention.  
 * 
 */
VectorXd  DQ_SerialManipulatorMDH::get_alphas() const
{
    return mdh_matrix_.row(3);
}

/**
 * @brief This method returns the fifth row of the Matrix mdh_matrix_, which
 *        corresponds to the type of joints of the robot.
 * 
 * @returns The fifth row of the Matrix mdh_matrix_, which corresponds to
 *          the type of joints of the robot.  
 * 
 */
VectorXd DQ_SerialManipulatorMDH::get_types() const
{
    return mdh_matrix_.row(4);
}

/**
 * @brief This method returns the first to_ith_link columns of the time derivative of the pose Jacobian.
 *        The base displacement and the effector are not taken into account.
 * @param q. VectorXd representing the robot joint configuration.
 * @param q_dot. VectorXd representing the robot joint velocities.
 * @param to_ith_link. The index to a link. This defines until which link the pose_jacobian_derivative
 *                     will be calculated.
 * @returns a MatrixXd representing the first to_ith_link columns of the desired Jacobian derivative.
 *
 */
MatrixXd DQ_SerialManipulatorMDH::raw_pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot, const int &to_ith_link) const
{
    _check_q_vec(q);
    _check_q_vec(q_dot);
    _check_to_ith_link(to_ith_link);

    int n = to_ith_link+1;
    DQ x_effector = raw_fkm(q,to_ith_link);
    MatrixXd J    = raw_pose_jacobian(q,to_ith_link);
    VectorXd vec_x_effector_dot = J*q_dot.head(n); //(to_ith_link);

    DQ x = DQ(1);
    MatrixXd J_dot = MatrixXd::Zero(8,n);
    int jth=0;

    for(int i=0;i<n;i++)
    {
        const DQ w = _get_w(i);
        const DQ z = 0.5*x*w*conj(x);

        VectorXd vec_zdot;
        if(i==0)
        {
            vec_zdot = VectorXd::Zero(8,1);
        }
        else
        {
            vec_zdot = 0.5*(haminus8(w*conj(x)) + hamiplus8(x*w)*C8())*raw_pose_jacobian(q,i-1)*q_dot.head(i);
        }

        J_dot.col(jth) = haminus8(x_effector)*vec_zdot + hamiplus8(z)*vec_x_effector_dot;
        x = x*_mdh2dq(q(jth),i);
        jth = jth+1;
    }

    return J_dot;
}


/**
 * @brief This method calculates the forward kinematic model and returns the dual quaternion
 *        corresponding to the last joint (the displacements due to the base and the effector
 *        are not taken into account).
 * @param q_vec. Vector of joint values.
 * @param to_ith_link. The index to a link. This defines until which link the raw_fkm will be calculated.
 * @returns The unit dual quaternion that represent the forward kinematic model until the link to_ith_link.
 * 
 */
DQ  DQ_SerialManipulatorMDH::raw_fkm(const VectorXd& q_vec, const int& to_ith_link) const
{
    _check_q_vec(q_vec);
    _check_to_ith_link(to_ith_link);

    DQ q(1);
    int j = 0;
    for (int i = 0; i < (to_ith_link+1); i++) {
        q = q * _mdh2dq(q_vec(i-j), i);
    }
    return q;
}


/**
 * @brief This method returns the pose Jacobian that satisfies vec(x_dot) = J * q_vec_dot, 
 *        where x = fkm(q_vec) and q_vec is the vector of joint variables. 
 *        This function does not take into account any base or end-effector displacements 
 *        and should be used mostly internally in the class.
 * @param q_vec. Vector of joint values.
 * @param to_ith_link. The index to a link. This defines until which link the raw_pose_jacobian 
 *                     will be calculated.
 * @returns The pose jacobian.
 * 
 */
MatrixXd DQ_SerialManipulatorMDH::raw_pose_jacobian(const VectorXd &q_vec, const int &to_ith_link) const
{
    _check_q_vec(q_vec);
    _check_to_ith_link(to_ith_link);

    MatrixXd J = MatrixXd::Zero(8,to_ith_link+1);
    DQ x_effector = raw_fkm(q_vec,to_ith_link);

    DQ x(1);

    for(int i=0;i<to_ith_link+1;i++)
    {
        DQ w = _get_w(i);
        DQ z = 0.5*Ad(x,w);
        x = x*_mdh2dq(q_vec(i),i);
        DQ j = z * x_effector;
        J.col(i)= vec8(j);
    }
    return J;
}


}
