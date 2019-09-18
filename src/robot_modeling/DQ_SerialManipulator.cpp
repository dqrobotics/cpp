/**
(C) Copyright 2011-2018 DQ Robotics Developers

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
- Mateus Rodrigues Martins (martinsrmateus@gmail.com)
*/

#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include <math.h>       //library for math functions
#include <stdexcept>    //For range_error
#include <limits>       //Used in pseudoinverse()
#include <string>

namespace DQ_robotics
{

/****************************************************************
**************DQ SERIALMANIPULATOR CLASS METHODS************************
*****************************************************************/

/**
* DQ_SerialManipulator constructor using boost matrix
*
* Returns a DQ_SerialManipulator object based on a 4xn or 5xn matrix named 'A' containing the Denavit-Hartenberg parameters.
* 'n' is the number of links of the robotic system. If 'A' has 4 rows there are no dummy joints. If 'A' has 4 rows,
* exists at least one dummy joint. 'A' matrix must be 4xn or 5xn. The 'A' rows 1 to 5 are respectively, 'theta' 1 to n,
* 'd' 1 to n, 'a' 1 to n, 'alpha' 1 to n and 'dummy joints' 1 to n parameters. The DH convention used is according to
* the 'type' parameter. 'type' is a string that can be 'standard' or 'modified' depending on the wanted convention. If
* something different of these values are attributed to 'type' parameter the standard convention is used as default.
* To create a DQ_SerialManipulator object using this, type: 'DQ dh_matrix__object(A,type)';

* \param MatrixXd A contain the Denavit-Hartenberg parameters for the kinematic model.
* \param std::string type contain the convention used in Denavit_Hartenberg.
*/
DQ_SerialManipulator::DQ_SerialManipulator(const MatrixXd& dh_matrix, const std::string& convention ):DQ_Kinematics()
{

    if (convention != "standard" && convention != "modified")
    {
        throw(std::range_error("Bad DQ_SerialManipulator(dh_matrix, convention) call: convention must be 'standard' or 'modified' "));
    }
    if (dh_matrix.rows() != 4 && dh_matrix.rows() != 5)
    {
        throw(std::range_error("Bad DQ_SerialManipulator(dh_matrix, convention) call: dh_matrix should be 5xn or 4xn"));
    }

    //dh_matrix_.resize(dh_matrix);
    dh_matrix_ = dh_matrix;
    curr_effector_ = DQ(1);
    dh_matrix_convention_ = convention;

    lower_q_limit_.resize(dh_matrix_.cols());
    upper_q_limit_.resize(dh_matrix_.cols());
}

// Public constant methods

/**

*/
MatrixXd DQ_SerialManipulator::getDHMatrix()
{
    MatrixXd DHMatrix = dh_matrix_;
    return DHMatrix;
}


/**
* Returns a constant int representing the number of links of a robotic system DQ_SerialManipulator object.
* It gets the number of columns of matrix 'A', passed to constructor and stored in the private attributte dh_matrix_.
* To use this member function, type: 'dh_matrix__object.links();'.
* \return A constant int.
*/
int  DQ_SerialManipulator::get_dim_configuration_space() const
{
    return dh_matrix_.cols();
}

VectorXd DQ_SerialManipulator::set_lower_q_limit(const VectorXd &lower_q_limit)
{
    if(lower_q_limit.size() != lower_q_limit_.size())
    {
        throw std::range_error(std::string("Error in set_lower_q_limit, size should be ")
                               + std::to_string(lower_q_limit_.size())
                               + std::string(" and not ")
                               + std::to_string(lower_q_limit.size()));
    }
    lower_q_limit_ = lower_q_limit;
}

VectorXd DQ_SerialManipulator::lower_q_limit() const
{
    return lower_q_limit_;
}

VectorXd DQ_SerialManipulator::set_upper_q_limit(const VectorXd &upper_q_limit)
{
    if(upper_q_limit.size() != upper_q_limit_.size())
    {
        throw std::range_error(std::string("Error in set_upper_q_limit, size should be ")
                               + std::to_string(upper_q_limit_.size())
                               + std::string(" and not ")
                               + std::to_string(upper_q_limit.size()));
    }
    upper_q_limit_ = upper_q_limit;
}

VectorXd DQ_SerialManipulator::upper_q_limit() const
{
    return upper_q_limit_;
}

/**
* Returns a constant vector representing the theta joint angles offset of a robotic system DQ_SerialManipulator object.
* It gets the first row of matrix 'A', passed to constructor and stored in the private attributte dh_matrix_.
* To use this member function, type: 'dh_matrix__object.theta();'.
* \return A constant Eigen::VectorXd (number of links).
*/
VectorXd  DQ_SerialManipulator::theta() const
{
    VectorXd aux_theta(dh_matrix_.cols());
    for (int i = 0; i < dh_matrix_.cols(); i++) {
        aux_theta(i) = dh_matrix_(0,i);
    }
    return aux_theta;
}



/**
* Returns a constant vector representing each 'd' offset along previous z to the common normal of a robotic system DQ_SerialManipulator object.
* It gets the second row of matrix 'A', passed to constructor and stored in the private attributte dh_matrix_.
* To use this member function, type: 'dh_matrix__object.d();'.
* \return A constant Eigen::VectorXd (number of links).
*/
VectorXd  DQ_SerialManipulator::d() const
{
    VectorXd aux_d(dh_matrix_.cols());
    for (int i = 0; i < dh_matrix_.cols(); i++) {
        aux_d(i) = dh_matrix_(1,i);
    }
    return aux_d;
}

/**
* Returns a constant vector representing each 'a' length of the common normal of a robotic system DQ_SerialManipulator object.
* It gets the third row of matrix 'A', passed to constructor and stored in the private attributte dh_matrix_.
* To use this member function, type: 'dh_matrix__object.a();'.
* \return A constant Eigen::VectorXd (number of links).
*/
VectorXd  DQ_SerialManipulator::a() const
{
    VectorXd aux_a(dh_matrix_.cols());
    for (int i = 0; i < dh_matrix_.cols(); i++) {
        aux_a(i) = dh_matrix_(2,i);
    }
    return aux_a;
}

/**
* Returns a constant vector representing each 'alpha' angle about common normal, from old z axis to new z axis of a
* robotic system DQ_SerialManipulator object. It gets the fourth row of matrix 'A', passed to constructor and stored in the private attributte
* dh_matrix_. To use this member function, type: 'dh_matrix__object.alpha();'.
* \return A constant Eigen::VectorXd (number of links).
*/
VectorXd  DQ_SerialManipulator::alpha() const
{
    VectorXd aux_alpha(dh_matrix_.cols());
    for (int i = 0; i < dh_matrix_.cols(); i++) {
        aux_alpha(i) = dh_matrix_(3,i);
    }
    return aux_alpha;
}

/**
* Returns a constant vector representing the existing 'dummy' axes of a robotic system DQ_SerialManipulator object.
* It gets the fifth row of matrix 'A', passed to constructor and stored in the private attributte dh_matrix_ when it exists.
* If not, dummy vector is a null with n elements. Being n equal to number of links. To use this member function, type:
* 'dh_matrix__object.dummy();'.
* \return A constant Eigen::VectorXd (number of links).
*/
VectorXd  DQ_SerialManipulator::dummy() const
{
    VectorXd aux_dummy(dh_matrix_.cols());
    if (dh_matrix_.rows() > 4){
        for (int i = 0; i < dh_matrix_.cols(); i++) {
            aux_dummy(i) = dh_matrix_(4,i);
        }
        return aux_dummy;
    }
    else {
        for (int i = 0; i < dh_matrix_.cols(); i++) {
            aux_dummy(i) = 0;
        }
        return aux_dummy;
    }
}


void DQ_SerialManipulator::set_dummy( const VectorXd& dummy_vector)
{

    if(dummy_vector.size() != dh_matrix_.cols())
    {
        std::cerr << std:: endl << "Cannot change dummy status: argument vector is of size = "
                  << dummy_vector.size() << " when it should be of size = " << dh_matrix_.cols() << std::endl;
        //Do nothing
        return;
    }

    if (dh_matrix_.rows() > 4){
        for (int i = 0; i < dh_matrix_.cols(); i++) {
            dh_matrix_(4,i) = dummy_vector(i);
        }
    }
    else{
        std::cerr << std::endl << "Kinematics body has no dummy information to change." << std::endl;
        //Do nothing
    }



}


/**
* Returns a constant int representing the number of 'dummy' axes of a robotic system DQ_SerialManipulator object.
* If there are no dummy axes the result is 0. To use this member function, type: 'dh_matrix__object.n_dummy();'.
* \return A constant int.
*/
int  DQ_SerialManipulator::n_dummy() const
{
    int aux_n_dummy = 0;
    if (dh_matrix_.rows() > 4){
        for (int i = 0; i < dh_matrix_.cols(); i++) {
            if(dh_matrix_(4,i) == 1.0)
                aux_n_dummy = aux_n_dummy + 1;
        }
        return aux_n_dummy;
    }
    else
        return aux_n_dummy;
}

/**
* Returns a constant std::string representing the Denavit Hartenberg convenction (standard or modified) used in a robotic system
* DQ_SerialManipulator object. To use this member function, type: 'dh_matrix__object.convention();'.
* \return A constant std::string.
*/
std::string  DQ_SerialManipulator::convention() const
{
    return dh_matrix_convention_;
}

/**
* Returns a constant DQ object representing current defined end effector of a robotic system DQ_SerialManipulator object.
* To use this member function, type: 'dh_matrix__object.effector();'.
* \return A constant DQ object.
*/
DQ  DQ_SerialManipulator::effector() const
{
    return curr_effector_;
}

/**
* Sets, by new_effector parameter, the pose of current end effector of a robotic system DQ_SerialManipulator object and returns it in a constant DQ object.
* To use this member function, type: 'dh_matrix__object.set_effector();'.
* \param DQ new_effector representing the new pose of robotic system end effector
* \return A constant DQ object.
*/
DQ  DQ_SerialManipulator::set_effector( const DQ& new_effector)
{
    curr_effector_ = new_effector;
    return curr_effector_;
}

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the last joint.
* The displacements due to the base and the effector are not taken into account. theta_vec is the vector of joint variables.
* This is an auxiliary function to be used mainly with the jacobian function.
* To use this member function, type: 'dh_matrix__object.raw_fkm(theta_vec);'.
* \param Eigen::VectorXd theta_vec is the vector representing the theta joint angles.
* \return A constant DQ object.
*/
DQ  DQ_SerialManipulator::raw_fkm( const VectorXd& theta_vec) const
{

    if(int(theta_vec.size()) != (this->get_dim_configuration_space() - this->n_dummy()) )
    {
        throw(std::range_error("Bad raw_fkm(theta_vec) call: Incorrect number of joint variables"));
    }

    DQ q(1);
    int j = 0;
    for (int i = 0; i < this->get_dim_configuration_space(); i++) {
        if(this->dummy()(i) == 1.0) {
            q = q * dh2dq(0.0, i+1);
            j = j + 1;
        }
        else
            q = q * dh2dq(theta_vec(i-j), i+1);
    }
    return q;
}

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the ith joint.
* The displacements due to the base and the effector are not taken into account. theta_vec is the vector of joint variables.
* This is an auxiliary function to be used mainly with the jacobian function.
* To use this member function, type: 'dh_matrix__object.raw_fkm(theta_vec, ith);'.
* \param Eigen::VectorXd theta_vec is the vector representing the theta joint angles.
* \param int ith is the position of the least joint included in the forward kinematic model
* \return A constant DQ object.
*/
DQ  DQ_SerialManipulator::raw_fkm( const VectorXd& theta_vec, const int& ith) const
{

    if(int(theta_vec.size()) != (this->get_dim_configuration_space() - this->n_dummy()) )
    {
        throw(std::range_error("Bad raw_fkm(theta_vec,ith) call: Incorrect number of joint variables"));
    }

    DQ q(1);
    int j = 0;
    for (int i = 0; i < ith; i++) {
        if(this->dummy()(i) == 1.0) {
            q = q * dh2dq(0, i+1);
            j = j + 1;
        }
        else
            q = q * dh2dq(theta_vec(i-j), i+1);
    }
    return q;
}

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the last joint.
* This function takes into account the displacement due to the base's and effector's poses. theta_vec is the vector of joint variables.
* To use this member function, type: 'dh_matrix__object.fkm(theta_vec);'.
* \param Eigen::VectorXd theta_vec is the vector representing the theta joint angles.
* \return A constant DQ object.
*/
DQ  DQ_SerialManipulator::fkm( const VectorXd& theta_vec) const
{
    DQ q = reference_frame_ * ( this->raw_fkm(theta_vec) ) * curr_effector_;
    return q;
}


/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the ith joint.
* This function takes into account the displacement due to the base's and effector's poses. theta_vec is the vector of joint variables.
* To use this member function, type: 'dh_matrix__object.fkm(theta_vec, ith);'.
* \param Eigen::VectorXd theta_vec is the vector representing the theta joint angles.
* \return A constant DQ object.
*/
DQ  DQ_SerialManipulator::fkm( const VectorXd& theta_vec, const int& ith) const
{
    DQ q = reference_frame_ * ( this->raw_fkm(theta_vec, ith) ) * curr_effector_;
    return q;
}

/** Returns the correspondent DQ object, for a given link's Denavit Hartenberg parameters.
* To use this member function type: 'dh_matrix__object.dh2dq(theta,i), where theta is the joint angle and i is the link number
* \param double theta_ang is the joint angle
* \param int link_i is the link number
* \return A constant DQ object
*/
DQ  DQ_SerialManipulator::dh2dq( const double& theta_ang, const int& link_i) const {

    Matrix<double,8,1> q(8);

    double d     = this->d()(link_i-1);
    double a     = this->a()(link_i-1);
    double alpha = this->alpha()(link_i-1);

    if(this->convention() == "standard") {

        q(0)=cos((theta_ang + this->theta()(link_i-1) )/2.0)*cos(alpha/2.0);
        q(1)=cos((theta_ang + this->theta()(link_i-1) )/2.0)*sin(alpha/2.0);
        q(2)=sin((theta_ang + this->theta()(link_i-1) )/2.0)*sin(alpha/2.0);
        q(3)=sin((theta_ang + this->theta()(link_i-1) )/2.0)*cos(alpha/2.0);
        double d2=d/2.0;
        double a2=a/2.0;
        q(4)= -d2*q(3) - a2*q(1);
        q(5)= -d2*q(2) + a2*q(0);
        q(6)=  d2*q(1) + a2*q(3);
        q(7)=  d2*q(0) - a2*q(2);
    }
    else{

        double h1 = cos((theta_ang + this->theta()(link_i-1) )/2.0)*cos(alpha/2.0);
        double h2 = cos((theta_ang + this->theta()(link_i-1) )/2.0)*sin(alpha/2.0);
        double h3 = sin((theta_ang + this->theta()(link_i-1) )/2.0)*sin(alpha/2.0);
        double h4 = sin((theta_ang + this->theta()(link_i-1) )/2.0)*cos(alpha/2.0);
        q(0)= h1;
        q(1)= h2;
        q(2)= -h3;
        q(3)= h4;
        double d2=d/2.0;
        double a2=a/2.0;
        q(4)=-d2*h4 - a2*h2;
        q(5)=-d2*h3 + a2*h1;
        q(6)=-(d2*h2 + a2*h4);
        q(7)=d2*h1 - a2*h3;
    }
    return DQ(q);
}


DQ  DQ_SerialManipulator::get_z( const VectorXd& q) const
{
    Matrix<double,8,1> z(8);

    z(0) = 0.0;
    z(1)=q(1)*q(3) + q(0)*q(2);
    z(2)=q(2)*q(3) - q(0)* q(1);
    z(3)=(q(3)*q(3)-q(2)*q(2)-q(1)*q(1)+q(0)*q(0))/2.0;
    z(4)=0.0;
    z(5)=q(1)*q(7)+q(5)*q(3)+q(0)*q(6)+q(4)*q(2);
    z(6)=q(2)*q(7)+q(6)*q(3)-q(0)*q(5)-q(4)*q(1);
    z(7)=q(3)*q(7)-q(2)*q(6)-q(1)*q(5)+q(0)*q(4);

    return DQ(z);
}


MatrixXd DQ_SerialManipulator::raw_pose_jacobian(const VectorXd& theta_vec, const int& to_link) const
{
    DQ q_effector = this->raw_fkm(theta_vec,to_link);
    DQ z;
    DQ q(1);

    MatrixXd J(8,(to_link - this->n_dummy()) );

    for (int i = 0; i < J.rows(); i++) {
        for(int j = 0; j < J.cols(); j++) {
            J(i,j) = 0;
        }
    }

    int ith = -1;
    for(int i = 0; i < to_link; i++) {

        // Use the standard DH convention
        if(this->convention() == "standard") {
            z = this->get_z(q.q);
        }
        // Use the modified DH convention
        else {
            DQ w(0, 0, -sin(this->alpha()(i)), cos(this->alpha()(i)), 0, 0, -this->a()(i)*cos(this->alpha()(i)), -this->a()(i)*sin(this->alpha()(i)));
            z =0.5 * q * w * q.conj();
        }
        if(this->dummy()(i) == 0.0) {
            q = q * this->dh2dq(theta_vec(ith+1), i+1);
            DQ aux_j = z * q_effector;
            for(int k = 0; k < J.rows(); k++) {
                J(k,ith+1) = aux_j.q(k);
            }
            ith = ith+1;
        }
        else
            // Dummy joints don't contribute to the Jacobian
            q = q * this->dh2dq(0.0,(i+1));
    }

    return J;
}

/** Returns a MatrixXd 8x(links - n_dummy) representing the Jacobian of a robotic system DQ_SerialManipulator object.
* theta_vec is the vector of joint variables.
* \param Eigen::VectorXd theta_vec is the vector representing the theta joint angles.
* \return A constant Eigen::MatrixXd (8,links - n_dummy).
*/
MatrixXd  DQ_SerialManipulator::pose_jacobian(const VectorXd& theta_vec, const int &to_link) const
{
    MatrixXd J = raw_pose_jacobian(theta_vec,to_link);
    if(to_link==this->get_dim_configuration_space())
    {
        J = hamiplus8(reference_frame_)*haminus8(curr_effector_)*J;
    }
    else
    {
        J = hamiplus8(reference_frame_)*J;
    }
    return J;
}

MatrixXd DQ_SerialManipulator::pose_jacobian(const VectorXd &theta_vec) const
{
    return pose_jacobian(theta_vec,get_dim_configuration_space());
}

MatrixXd DQ_SerialManipulator::pose_jacobian_derivative(const VectorXd &theta_vec, const VectorXd &theta_vec_dot, const int &to_link) const
{

    int n = to_link;
    DQ x_effector = raw_fkm(theta_vec,to_link);
    MatrixXd J    = raw_pose_jacobian(theta_vec,to_link);
    VectorXd vec_x_effector_dot = J*theta_vec_dot.head(to_link);

    DQ x = DQ(1);
    MatrixXd J_dot = MatrixXd::Zero(8,n-n_dummy());
    int jth=0;

    for(int i=0;i<n;i++)
    {
        DQ w;
        DQ z;
        // Use the standard DH convention
        if(this->convention() == "standard") {
            w = k_;
            z = get_z(x.q);
        }
        else //Use the modified DH convention
        {
            w = DQ(0,0,-sin(alpha()(i)),cos(alpha()(i)),0,0,-a()(i)*cos(alpha()(i)),-a()(i)*sin(alpha()(i)));
            z = 0.5*x*w*conj(x);
        }

        if( dummy()(i)!=1.0 )
        {
            VectorXd vec_zdot = 0.5*(haminus8(w*conj(x)) + hamiplus8(x*w)*C8())*raw_pose_jacobian(theta_vec,i)*theta_vec_dot.head(i);

            J_dot.col(jth) = haminus8(x_effector)*vec_zdot + hamiplus8(z)*vec_x_effector_dot;
            x = x*dh2dq(theta_vec(jth),i+1);
            jth = jth+1;
        }
        else
        {
            //Dummy joints don't contribute to the Jacobian
            x = x*dh2dq(0,i);
        }
    }

    return J_dot;
}

}//namespace DQ_robotics

