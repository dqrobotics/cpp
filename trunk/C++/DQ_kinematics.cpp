#include "DQ_kinematics.h"
#include "DQ.h"

namespace DQ_robotics
{

/****************************************************************
**************NAMESPACE ONLY FUNCTIONS***************************
*****************************************************************/

/**
*/
int links( const DQ_kinematics& dq_kin)
{
    return dq_kin.links();
}

/**
* Returns a constant vector representing the theta joint angles offset of a robotic system.
*/
VectorXd  theta( const DQ_kinematics& dq_kin)
{
    return dq_kin.theta();
}

/**
* Returns a constant vector representing each 'd' offset along previous z to the common normal of a robotic system DQ_kinematics object.
*/
VectorXd  d( const DQ_kinematics& dq_kin)
{
    return dq_kin.d();
}

/**
* Returns a constant vector representing each 'a' length of the common normal of a robotic system DQ_kinematics object.
*/
VectorXd  a( const DQ_kinematics& dq_kin)
{
    return dq_kin.a();
}

/**
* Returns a constant vector representing each 'alpha' angle about common normal, from old z axis to new z axis of a
* robotic system DQ_kinematics object.
*/
VectorXd  alpha( const DQ_kinematics& dq_kin)
{
    return dq_kin.alpha();
}

/**
* Returns a constant vector representing the existing 'dummy' axes of a robotic system DQ_kinematics object.
*/
VectorXd  dummy( const DQ_kinematics& dq_kin)
{
    return dq_kin.dummy();
}

/**
* Returns a constant int representing the number of 'dummy' axes of a robotic system DQ_kinematics object.
*/
int  n_dummy( const DQ_kinematics& dq_kin)
{
    return dq_kin.n_dummy();
}

/**
* Returns a constant std::string representing the Denavit Hartenberg convenction (standard or modified) used in a robotic system
* DQ_kinematics object.
*/
std::string  convention( const DQ_kinematics& dq_kin)
{
    return dq_kin.convention();
}

/**
* Returns a constant DQ object representing current defined base of a robotic system DQ_kinematics object.
*/
DQ  base( const DQ_kinematics& dq_kin)
{
    return dq_kin.base();
}

/**
* Returns a constant DQ object representing current defined end effector of a robotic system DQ_kinematics object.
*/
DQ  effector( const DQ_kinematics& dq_kin)
{
    return dq_kin.effector();
}

/**
* Returns a constant MatrixXd 8x8 dimensions representing C8, a diagonal negative unit matrix.
* Given the jacobian matrix J that satisfies 'vec8(dot_x) = J * dot_theta', where dot_x is the time derivative of the translation
* quaternion and dot_theta is the time derivative of the joint vector, the following relation
* is established: 'vec8(dot_x) = C8 * J * dot_theta'.
* \return A constant Eigen::MatrixXd (8,8).
*/
Matrix<double,8,8>  C8()
{

    Matrix<double,8,8> diag_C8 = Matrix<double,8,8>::Zero();

    diag_C8(0,0) =  1;
    diag_C8(1,1) = -1;
    diag_C8(2,2) = -1;
    diag_C8(3,3) = -1;
    diag_C8(4,4) =  1;
    diag_C8(5,5) = -1;
    diag_C8(6,6) = -1;
    diag_C8(7,7) = -1;

    return diag_C8;
}

/**
* Returns a constant MatrixXd 4x4 dimensions representing C4, a diagonal negative unit matrix.
* Given the jacobian matrix J that satisfies 'vec4(dot_x) = J * dot_theta', where dot_x is the time derivative of the translation
* quaternion and dot_theta is the time derivative of the joint vector, the following relation
* is established: 'vec4(dot_x) = C4 * J * dot_theta'.
* \return A constant Eigen::MatrixXd (4,4).

*/
Matrix<double,4,4>  C4()
{

    Matrix<double,4,4> diag_C4 = Matrix<double,4,4>::Zero();

    diag_C4(0,0) =  1;
    diag_C4(1,1) = -1;
    diag_C4(2,2) = -1;
    diag_C4(3,3) = -1;

    return diag_C4;
}

/**
* Sets, by new_base parameter, the pose of current base of a robotic system DQ_kinematics object and returns it in a constant DQ object.
*/
DQ  set_base( DQ_kinematics& dq_kin, const DQ& new_base)
{
    return dq_kin.set_base(new_base);
}


/**
* Sets, by new_effector parameter, the pose of current end effector of a robotic system DQ_kinematics object and returns it in a constant DQ object.
*/
DQ  set_effector(DQ_kinematics& dq_kin, const DQ& new_effector)
{
    return dq_kin.set_effector(new_effector);
}

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the last joint.
*/
DQ  raw_fkm( const DQ_kinematics& dq_kin, const VectorXd& theta_vec)
{
    return dq_kin.raw_fkm(theta_vec);
}

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the ith joint.
*/
DQ  raw_fkm( const DQ_kinematics& dq_kin, const VectorXd& theta_vec, const int& ith)
{
    return dq_kin.raw_fkm(theta_vec, ith);
}


/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the last joint
*/
DQ  fkm(DQ_kinematics dq_kin, VectorXd theta_vec)
{
    return dq_kin.fkm(theta_vec);
}

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the ith joint.

*/
DQ  fkm(DQ_kinematics dq_kin, VectorXd theta_vec, int ith)
{
    return dq_kin.fkm(theta_vec, ith);
}


/** Returns the correspondent DQ object, for a given link's Denavit Hartenberg parameters.
*/
DQ  dh2dq( const DQ_kinematics& dq_kin, const double& theta_ang, const int& link_i)
{
    return dq_kin.dh2dq(theta_ang, link_i);
}

DQ  get_z( const DQ_kinematics& dq_kin, const VectorXd& q)
{
    return dq_kin.get_z(q);
}

/** 
Returns a MatrixXd 8x(links - n_dummy) representing the Jacobian of a robotic system DQ_kinematics object.
*/
MatrixXd  jacobian( const DQ_kinematics& dq_kin, const VectorXd& theta_vec)
{
    return dq_kin.jacobian(theta_vec);
}

/** 
Returns a MatrixXd 8x(links - n_dummy) representing the Jacobian of a robotic system DQ_kinematics object.
*/
MatrixXd  analyticalJacobian( const DQ_kinematics& dq_kin, const VectorXd& theta_vec)
{
    return dq_kin.jacobian(theta_vec);
}


/**
* Obtains the rotation jacobian, relating the derivative of the rotation quaternion to the derivative of the joint variables as being the first 4 rows of the analytical jacobian.
* \param MatrixXd analytical_jacobian The robot analytical jacobian.
* \return The rotation jacobian.
*/
MatrixXd  rotationJacobian( const MatrixXd& analytical_jacobian)
{
	return analytical_jacobian.block(0,0,4,analytical_jacobian.cols());
}

/** Returns the translation Jacobian; that it, the Jacobian that satisfies the relation dot_p = Jp * dot_theta.
* Where dot_p is the time derivative of the translation quaternion and dot_theta is the time derivative of the joint vector.
* To use this member function type: 'dh_matrix__object.jacobp(param_jacobian, x).
* \param Eigen::MatrixXd param_jacobian is the Jacobian of a robotic system
* \param Eigen::Matrix<double,8,1> x is the vector which constructs a translation DQ object
* \return A constant Eigen::MatrixXd
*/
MatrixXd  jacobp( const MatrixXd& analytical_jacobian, const Matrix<double,8,1>& x)
{
    DQ dq_x(x);
    DQ dq_x_conj_P = dq_x.P();
    dq_x_conj_P = dq_x_conj_P.conj();
    MatrixXd aux_J1(4,analytical_jacobian.cols());
    MatrixXd aux_J2(4,analytical_jacobian.cols());
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < analytical_jacobian.cols(); j++) {
            aux_J1(i,j) = analytical_jacobian(i,j);
            aux_J2(i,j) = analytical_jacobian((i+4),j);
        }
    }
    MatrixXd aux = Hplus4(dq_x.D())*C4();
    MatrixXd Jp = 2*(Hminus4(dq_x_conj_P)*aux_J2) + 2*(aux*aux_J1);
    return Jp;
}

MatrixXd  translationJacobian( const MatrixXd& analytical_jacobian, const Matrix<double,8,1>& x)
{
    return DQ_robotics::jacobp(analytical_jacobian,x);
}

/** Returns the distance Jacobian; that it, the Jacobian that satisfies the relation dot_(d^2) = Jd * dot_theta.
* where dot_(d^2) is the time derivative of the square of the distance between the end-effector and the base and dot_theta is
* the time derivative of the joint vector.
* To use this member function type: 'dh_matrix__object.jacobd(param_jacobian, x).
* \param Eigen::MatrixXd param_jacobian is the Jacobian of a robotic system
* \param Eigen::Matrix<double,8,1> x is the vector which constructs a translation DQ object
* \return A constant Eigen::MatrixXd
*/
MatrixXd  jacobd( const MatrixXd& param_jacobian, const Matrix<double,8,1>& x)
{
    DQ dq_x(x);
    DQ p = translation(dq_x);
    MatrixXd Jp = jacobp(param_jacobian, x);
    MatrixXd vec4p_T(1,4);
    for (int i = 0; i < 4; i++) {
        vec4p_T(0,i) = vec4(p)(i,0);
    }
    MatrixXd Jd = 2 * (vec4p_T * Jp);

    return Jd;
}

MatrixXd  distanceJacobian( const MatrixXd& param_jacobian, const Matrix<double,8,1>& x)
{
    return DQ_robotics::jacobd(param_jacobian,x);
}

/**
* Pseudo inverse implementation mimicking the on in MATLAB.
* \param Eigen::MatrixXd matrix: the matrix whose pseudoinverse you want.
* \return A constant Eigen::MatrixXd that is the Moore-Penrose pseudoinverse of matrix with the default MATLAB tolerance.
* \see  http://www.mathworks.com/help/matlab/ref/pinv.html
*/
MatrixXd  pseudoInverse( const MatrixXd& matrix)
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



/****************************************************************
**************DQ KINEMATICS CLASS METHODS************************
*****************************************************************/


/**
* DQ_kinematics constructor using boost matrix
*
* Returns a DQ_kinematics object based on a 4xn or 5xn matrix named 'A' containing the Denavit-Hartenberg parameters.
* 'n' is the number of links of the robotic system. If 'A' has 4 rows there are no dummy joints. If 'A' has 4 rows,
* exists at least one dummy joint. 'A' matrix must be 4xn or 5xn. The 'A' rows 1 to 5 are respectively, 'theta' 1 to n,
* 'd' 1 to n, 'a' 1 to n, 'alpha' 1 to n and 'dummy joints' 1 to n parameters. The DH convention used is according to
* the 'type' parameter. 'type' is a string that can be 'standard' or 'modified' depending on the wanted convention. If
* something different of these values are attributed to 'type' parameter the standard convention is used as default.
* To create a DQ_kinematics object using this, type: 'DQ dh_matrix__object(A,type)';

* \param MatrixXd A contain the Denavit-Hartenberg parameters for the kinematic model.
* \param std::string type contain the convention used in Denavit_Hartenberg.
*/
DQ_kinematics::DQ_kinematics(const MatrixXd& dh_matrix, const std::string& convention ){
 
    if (convention != "standard" && convention != "modified")
    {
        throw(std::range_error("Bad DQ_kinematics(dh_matrix, convention) call: convention must be 'standard' or 'modified' "));
    }
	  if (dh_matrix.rows() != 4 && dh_matrix.rows() != 5)
    {
        throw(std::range_error("Bad DQ_kinematics(dh_matrix, convention) call: dh_matrix should be 5xn or 4xn"));
    }

    //dh_matrix_.resize(dh_matrix);
    dh_matrix_ = dh_matrix;
    curr_base_ = DQ(1);
    curr_effector_ = DQ(1);
    dh_matrix_convention_ = convention;
};

/**
* DQ_kinematics Destructor
*
* Deletes from memory the DQ_kinematics object caller. To use this destructor, type: 'dh_matrix__object.~DQ_kinematics();'. Dont need parameters.
*/
DQ_kinematics::~DQ_kinematics(){};

// Public constant methods

/**

*/
MatrixXd DQ_kinematics::getDHMatrix()
{
    MatrixXd DHMatrix = dh_matrix_;
    return DHMatrix;
}


/**
* Returns a constant int representing the number of links of a robotic system DQ_kinematics object.
* It gets the number of columns of matrix 'A', passed to constructor and stored in the private attributte dh_matrix_.
* To use this member function, type: 'dh_matrix__object.links();'.
* \return A constant int.
*/
int  DQ_kinematics::links() const
{
    return dh_matrix_.cols();
};

/**
* Returns a constant vector representing the theta joint angles offset of a robotic system DQ_kinematics object.
* It gets the first row of matrix 'A', passed to constructor and stored in the private attributte dh_matrix_.
* To use this member function, type: 'dh_matrix__object.theta();'.
* \return A constant Eigen::VectorXd (number of links).
*/
VectorXd  DQ_kinematics::theta() const
{
    VectorXd aux_theta(dh_matrix_.cols());
    for (int i = 0; i < dh_matrix_.cols(); i++) {
        aux_theta(i) = dh_matrix_(0,i);
    }
    return aux_theta;
};



/**
* Returns a constant vector representing each 'd' offset along previous z to the common normal of a robotic system DQ_kinematics object.
* It gets the second row of matrix 'A', passed to constructor and stored in the private attributte dh_matrix_.
* To use this member function, type: 'dh_matrix__object.d();'.
* \return A constant Eigen::VectorXd (number of links).
*/
VectorXd  DQ_kinematics::d() const
{
    VectorXd aux_d(dh_matrix_.cols());
    for (int i = 0; i < dh_matrix_.cols(); i++) {
        aux_d(i) = dh_matrix_(1,i);
    }
    return aux_d;
};

/**
* Returns a constant vector representing each 'a' length of the common normal of a robotic system DQ_kinematics object.
* It gets the third row of matrix 'A', passed to constructor and stored in the private attributte dh_matrix_.
* To use this member function, type: 'dh_matrix__object.a();'.
* \return A constant Eigen::VectorXd (number of links).
*/
VectorXd  DQ_kinematics::a() const
{
    VectorXd aux_a(dh_matrix_.cols());
    for (int i = 0; i < dh_matrix_.cols(); i++) {
        aux_a(i) = dh_matrix_(2,i);
    }
    return aux_a;
};

/**
* Returns a constant vector representing each 'alpha' angle about common normal, from old z axis to new z axis of a
* robotic system DQ_kinematics object. It gets the fourth row of matrix 'A', passed to constructor and stored in the private attributte
* dh_matrix_. To use this member function, type: 'dh_matrix__object.alpha();'.
* \return A constant Eigen::VectorXd (number of links).
*/
VectorXd  DQ_kinematics::alpha() const
{
    VectorXd aux_alpha(dh_matrix_.cols());
    for (int i = 0; i < dh_matrix_.cols(); i++) {
        aux_alpha(i) = dh_matrix_(3,i);
    }
    return aux_alpha;
};

/**
* Returns a constant vector representing the existing 'dummy' axes of a robotic system DQ_kinematics object.
* It gets the fifth row of matrix 'A', passed to constructor and stored in the private attributte dh_matrix_ when it exists.
* If not, dummy vector is a null with n elements. Being n equal to number of links. To use this member function, type:
* 'dh_matrix__object.dummy();'.
* \return A constant Eigen::VectorXd (number of links).
*/
VectorXd  DQ_kinematics::dummy() const
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
};


void DQ_kinematics::set_dummy( const VectorXd& dummy_vector)
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
* Returns a constant int representing the number of 'dummy' axes of a robotic system DQ_kinematics object.
* If there are no dummy axes the result is 0. To use this member function, type: 'dh_matrix__object.n_dummy();'.
* \return A constant int.
*/
int  DQ_kinematics::n_dummy() const
{
    int aux_n_dummy = 0;
    if (dh_matrix_.rows() > 4){
        for (int i = 0; i < dh_matrix_.cols(); i++) {
            if(dh_matrix_(4,i) == 1)
                aux_n_dummy = aux_n_dummy + 1;
        }
        return aux_n_dummy;
    }
    else
        return aux_n_dummy;
};

/**
* Returns a constant std::string representing the Denavit Hartenberg convenction (standard or modified) used in a robotic system
* DQ_kinematics object. To use this member function, type: 'dh_matrix__object.convention();'.
* \return A constant std::string.
*/
std::string  DQ_kinematics::convention() const
{
    return dh_matrix_convention_;
};

/**
* Returns a constant DQ object representing current defined base of a robotic system DQ_kinematics object.
* To use this member function, type: 'dh_matrix__object.base();'.
* \return A constant DQ object.
*/
DQ  DQ_kinematics::base() const
{
    return curr_base_;
};

/**
* Returns a constant DQ object representing current defined end effector of a robotic system DQ_kinematics object.
* To use this member function, type: 'dh_matrix__object.effector();'.
* \return A constant DQ object.
*/
DQ  DQ_kinematics::effector() const
{
    return curr_effector_;
};

/**
* Sets, by new_base parameter, the pose of current base of a robotic system DQ_kinematics object and returns it in a constant DQ object.
* To use this member function, type: 'dh_matrix__object.set_base();'.
* \param DQ new_base representing the new pose of robotic system base
* \return A constant DQ object.
*/
DQ  DQ_kinematics::set_base( const DQ& new_base)
{
    curr_base_ = new_base;
    return curr_base_;
};

/**
* Sets, by new_effector parameter, the pose of current end effector of a robotic system DQ_kinematics object and returns it in a constant DQ object.
* To use this member function, type: 'dh_matrix__object.set_effector();'.
* \param DQ new_effector representing the new pose of robotic system end effector
* \return A constant DQ object.
*/
DQ  DQ_kinematics::set_effector( const DQ& new_effector)
{
    curr_effector_ = new_effector;
    return curr_effector_;
};

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the last joint.
* The displacements due to the base and the effector are not taken into account. theta_vec is the vector of joint variables.
* This is an auxiliary function to be used mainly with the jacobian function.
* To use this member function, type: 'dh_matrix__object.raw_fkm(theta_vec);'.
* \param Eigen::VectorXd theta_vec is the vector representing the theta joint angles.
* \return A constant DQ object.
*/
DQ  DQ_kinematics::raw_fkm( const VectorXd& theta_vec) const
{

    if((int)theta_vec.size() != (this->links() - this->n_dummy()) )
    {
        throw(std::range_error("Bad raw_fkm(theta_vec) call: Incorrect number of joint variables"));
    }

    DQ q(1);
    int j = 0;
    for (int i = 0; i < this->links(); i++) {
        if(this->dummy()(i) == 1) {
            q = q * DQ_robotics::dh2dq((*this), 0.0, i+1);
            j = j + 1;
        }
        else
            q = q * DQ_robotics::dh2dq((*this), theta_vec(i-j), i+1);
    }
    return q;
};

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the ith joint.
* The displacements due to the base and the effector are not taken into account. theta_vec is the vector of joint variables.
* This is an auxiliary function to be used mainly with the jacobian function.
* To use this member function, type: 'dh_matrix__object.raw_fkm(theta_vec, ith);'.
* \param Eigen::VectorXd theta_vec is the vector representing the theta joint angles.
* \param int ith is the position of the least joint included in the forward kinematic model
* \return A constant DQ object.
*/
DQ  DQ_kinematics::raw_fkm( const VectorXd& theta_vec, const int& ith) const
{

    if((int)theta_vec.size() != (this->links() - this->n_dummy()) )
    {
        throw(std::range_error("Bad raw_fkm(theta_vec,ith) call: Incorrect number of joint variables"));
    }

    DQ q(1);
    int j = 0;
    for (int i = 0; i < ith; i++) {
        if(this->dummy()(i) == 1) {
            q = q * DQ_robotics::dh2dq( (*this) , 0, i+1);
            j = j + 1;
        }
        else
            q = q * DQ_robotics::dh2dq( (*this) , theta_vec(i-j), i+1);
    }
    return q;
};

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the last joint.
* This function takes into account the displacement due to the base's and effector's poses. theta_vec is the vector of joint variables.
* To use this member function, type: 'dh_matrix__object.fkm(theta_vec);'.
* \param Eigen::VectorXd theta_vec is the vector representing the theta joint angles.
* \return A constant DQ object.
*/
DQ  DQ_kinematics::fkm( const VectorXd& theta_vec) const
{
    DQ q = curr_base_ * ( this->raw_fkm(theta_vec) ) * curr_effector_;
    return q;
};


/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the ith joint.
* This function takes into account the displacement due to the base's and effector's poses. theta_vec is the vector of joint variables.
* To use this member function, type: 'dh_matrix__object.fkm(theta_vec, ith);'.
* \param Eigen::VectorXd theta_vec is the vector representing the theta joint angles.
* \return A constant DQ object.
*/
DQ  DQ_kinematics::fkm( const VectorXd& theta_vec, const int& ith) const
{
    DQ q = curr_base_ * ( this->raw_fkm(theta_vec, ith) ) * curr_effector_;
    return q;
};

/** Returns the correspondent DQ object, for a given link's Denavit Hartenberg parameters.
* To use this member function type: 'dh_matrix__object.dh2dq(theta,i), where theta is the joint angle and i is the link number
* \param double theta_ang is the joint angle
* \param int link_i is the link number
* \return A constant DQ object
*/
DQ  DQ_kinematics::dh2dq( const double& theta_ang, const int& link_i) const {

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
};


DQ  DQ_kinematics::get_z( const VectorXd& q) const
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
};

/** Returns a MatrixXd 8x(links - n_dummy) representing the Jacobian of a robotic system DQ_kinematics object.
* theta_vec is the vector of joint variables.
* \param Eigen::VectorXd theta_vec is the vector representing the theta joint angles.
* \return A constant Eigen::MatrixXd (8,links - n_dummy).
*/
MatrixXd  DQ_kinematics::jacobian( const VectorXd& theta_vec) const
{

    DQ q_effector = this->raw_fkm(theta_vec);

    DQ z;
    DQ q(1);

    MatrixXd J(8,(this->links() - this->n_dummy()) );

    for (int i = 0; i < J.rows(); i++) {
        for(int j = 0; j < J.cols(); j++) {
            J(i,j) = 0;
        }
    }
    int ith = -1;
    for(int i = 0; i < this->links(); i++) {

            // Use the standard DH convention
            if(this->convention() == "standard") {
                z = this->get_z(q.q);
            }
            // Use the modified DH convention
            else {
                DQ w(0, 0, -sin(this->alpha()(i)), cos(this->alpha()(i)), 0, 0, -this->a()(i)*cos(this->alpha()(i)), -this->a()(i)*sin(this->alpha()(i)));
                z =0.5 * q * w * q.conj();
            }

            if(this->dummy()(i) == 0) {
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

    // Takes the base's displacement into account
    Matrix<double,8,8> aux_J(8,8);
    aux_J = Hminus8(curr_effector_);
    aux_J = Hplus8(curr_base_)*aux_J;
    J = aux_J*J;

    return J;
};

MatrixXd DQ_kinematics::analyticalJacobian( const VectorXd& theta_vec) const
{
    return DQ_kinematics::jacobian(theta_vec);
}




}//namespace DQ_robotics
