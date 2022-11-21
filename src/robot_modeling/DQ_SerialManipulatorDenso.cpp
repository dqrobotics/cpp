#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDenso.h>
namespace DQ_robotics
{

DQ_SerialManipulatorDenso::DQ_SerialManipulatorDenso(const MatrixXd& denso_matrix):
    DQ_SerialManipulator(denso_matrix.cols())
{
    if(denso_matrix.rows() != 6)
    {
        throw(std::range_error("Bad DQ_SerialManipulatorDenso(MatrixXd) call: denso_matrix should be 6xn"));
    }
    denso_matrix_ = denso_matrix;
}

DQ DQ_SerialManipulatorDenso::_denso2dh(const double &q, const int &ith) const
{
    const double& a = denso_matrix_(0,ith);
    const double& b = denso_matrix_(1,ith);
    const double& d = denso_matrix_(2,ith);
    const double& alpha = denso_matrix_(3,ith);
    const double& beta = denso_matrix_(4,ith);
    const double& gamma = denso_matrix_(5,ith);

    DQ z_rot = cos((gamma + q)/2.0) + k_*sin((gamma + q)/2.0);
    DQ q_t     = 1.0+0.5*E_*(a*i_+b*j_+d*k_);
    DQ q_alpha = cos(alpha/2.)+i_*sin(alpha/2.);
    DQ q_beta  = cos(beta/2.)+j_*sin(beta/2.);

    return z_rot*q_t*q_alpha*q_beta;
}

VectorXd DQ_SerialManipulatorDenso::get_as() const
{
    return denso_matrix_.row(0);
}

VectorXd DQ_SerialManipulatorDenso::get_bs() const
{
    return denso_matrix_.row(1);
}

VectorXd DQ_SerialManipulatorDenso::get_ds() const
{
    return denso_matrix_.row(2);
}

VectorXd DQ_SerialManipulatorDenso::get_alphas() const
{
    return denso_matrix_.row(3);
}

VectorXd DQ_SerialManipulatorDenso::get_betas() const
{
    return denso_matrix_.row(4);
}

VectorXd DQ_SerialManipulatorDenso::get_gammas() const
{
    return denso_matrix_.row(5);
}

DQ  DQ_SerialManipulatorDenso::raw_fkm(const VectorXd& q_vec, const int& to_ith_link) const
{
    _check_q_vec(q_vec);
    _check_to_ith_link(to_ith_link);

    DQ q(1);
    int j = 0;
    for (int i = 0; i < (to_ith_link+1); i++) {
        q = q * _denso2dh(q_vec(i-j), i);
    }
    return q;
}

MatrixXd DQ_SerialManipulatorDenso::raw_pose_jacobian(const VectorXd &q_vec, const int &to_ith_link) const
{
    _check_q_vec(q_vec);
    _check_to_ith_link(to_ith_link);

    MatrixXd J = MatrixXd::Zero(8,to_ith_link+1);
    DQ x_effector = raw_fkm(q_vec,to_ith_link);

    DQ x(1);

    for(int i=0;i<to_ith_link+1;i++)
    {
        const DQ w = k_;
        const DQ z = 0.5*Ad(x,w);
        x = x*_denso2dh(q_vec(i),i);
        const DQ j = z * x_effector;
        J.col(i)= vec8(j);
    }
    return J;
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
MatrixXd DQ_SerialManipulatorDenso::raw_pose_jacobian_derivative(const VectorXd &q, const VectorXd &q_dot, const int &to_ith_link) const
{
    throw std::runtime_error(std::string("pose_jacobian_derivative is not implemented yet."));
}



}
