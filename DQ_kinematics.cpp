#include "DQ_kinematics.h"
#include "DQ.h"
#include <iostream>
#include <iomanip>
#include <math.h>

//For the pseudoinverse calculation
#include <limits>

using std::cout;

namespace DQ_robotics
{

/****************************************************************
**************NAMESPACE ONLY FUNCTIONS***************************
*****************************************************************/

/**
*/
int const links(DQ_kinematics param_dq_kin)
{
    return param_dq_kin.links();
}

/**
* Returns a constant vector representing the theta joint angles offset of a robotic system.
*/
VectorXd const theta(DQ_kinematics param_dq_kin)
{
    return param_dq_kin.theta();
}

/**
* Returns a constant vector representing each 'd' offset along previous z to the common normal of a robotic system DQ_kinematics object.
*/
VectorXd const d(DQ_kinematics param_dq_kin)
{
    return param_dq_kin.d();
}

/**
* Returns a constant vector representing each 'a' length of the common normal of a robotic system DQ_kinematics object.
*/
VectorXd const a(DQ_kinematics param_dq_kin)
{
    return param_dq_kin.a();
}

/**
* Returns a constant vector representing each 'alpha' angle about common normal, from old z axis to new z axis of a
* robotic system DQ_kinematics object.
*/
VectorXd const alpha(DQ_kinematics param_dq_kin)
{
    return param_dq_kin.alpha();
}

/**
* Returns a constant vector representing the existing 'dummy' axes of a robotic system DQ_kinematics object.
*/
VectorXd const dummy(DQ_kinematics param_dq_kin)
{
    return param_dq_kin.dummy();
}

/**
* Returns a constant int representing the number of 'dummy' axes of a robotic system DQ_kinematics object.
*/
int const n_dummy(DQ_kinematics param_dq_kin)
{
    return param_dq_kin.n_dummy();
}

/**
* Returns a constant std::string representing the Denavit Hartenberg convenction (standard or modified) used in a robotic system
* DQ_kinematics object.
*/
std::string const convention(DQ_kinematics param_dq_kin)
{
    return param_dq_kin.convention();
}

/**
* Returns a constant DQ object representing current defined base of a robotic system DQ_kinematics object.
*/
DQ const base(DQ_kinematics param_dq_kin)
{
    return param_dq_kin.base();
}

/**
* Returns a constant DQ object representing current defined end effector of a robotic system DQ_kinematics object.
*/
DQ const effector(DQ_kinematics param_dq_kin)
{
    return param_dq_kin.effector();
}

/**
* Returns a constant MatrixXd 8x8 dimensions representing C8, a diagonal negative unit matrix.
* Given the jacobian matrix J that satisfies 'vec8(dot_x) = J * dot_theta', where dot_x is the time derivative of the translation
* quaternion and dot_theta is the time derivative of the joint vector, the following relation
* is established: 'vec8(dot_x) = C8 * J * dot_theta'.
* \return A constant Eigen::MatrixXd (8,8).
*/
Matrix<double,8,8> const C8()
{

    Matrix<double,8,8> diag_C8(8,8);

    diag_C8(0,0) = 1; diag_C8(0,1) = 0; diag_C8(0,2) = 0; diag_C8(0,3) = 0;
    diag_C8(1,0) = 0; diag_C8(1,1) = -1; diag_C8(1,2) = 0; diag_C8(1,3) = 0;
    diag_C8(2,0) = 0; diag_C8(2,1) = 0; diag_C8(2,2) = -1; diag_C8(2,3) = 0;
    diag_C8(3,0) = 0; diag_C8(3,1) = 0; diag_C8(3,2) = 0; diag_C8(3,3) = -1;

    diag_C8(0,4) = 0; diag_C8(0,5) = 0; diag_C8(0,6) = 0; diag_C8(0,7) = 0;
    diag_C8(1,4) = 0; diag_C8(1,5) = 0; diag_C8(1,6) = 0; diag_C8(1,7) = 0;
    diag_C8(2,4) = 0; diag_C8(2,5) = 0; diag_C8(2,6) = 0; diag_C8(2,7) = 0;
    diag_C8(3,4) = 0; diag_C8(3,5) = 0; diag_C8(3,6) = 0; diag_C8(3,7) = 0;

    diag_C8(4,0) = 0; diag_C8(4,1) = 0; diag_C8(4,2) = 0; diag_C8(4,3) = 0;
    diag_C8(5,0) = 0; diag_C8(5,1) = 0; diag_C8(5,2) = 0; diag_C8(5,3) = 0;
    diag_C8(6,0) = 0; diag_C8(6,1) = 0; diag_C8(6,2) = 0; diag_C8(6,3) = 0;
    diag_C8(7,0) = 0; diag_C8(7,1) = 0; diag_C8(7,2) = 0; diag_C8(7,3) = 0;

    diag_C8(4,4) = 1; diag_C8(4,5) = 0; diag_C8(4,6) = 0; diag_C8(4,7) = 0;
    diag_C8(5,4) = 0; diag_C8(5,5) = -1; diag_C8(5,6) = 0; diag_C8(5,7) = 0;
    diag_C8(6,4) = 0; diag_C8(6,5) = 0; diag_C8(6,6) = -1; diag_C8(6,7) = 0;
    diag_C8(7,4) = 0; diag_C8(7,5) = 0; diag_C8(7,6) = 0; diag_C8(7,7) = -1;

    return diag_C8;
}

/**
* Returns a constant MatrixXd 4x4 dimensions representing C4, a diagonal negative unit matrix.
* Given the jacobian matrix J that satisfies 'vec4(dot_x) = J * dot_theta', where dot_x is the time derivative of the translation
* quaternion and dot_theta is the time derivative of the joint vector, the following relation
* is established: 'vec4(dot_x) = C4 * J * dot_theta'.
* \return A constant Eigen::MatrixXd (4,4).

*/
Matrix<double,4,4> const C4()
{

    Matrix<double,4,4> diag_C4(4,4);

    diag_C4 << 1, 0, 0, 0,
               0,-1, 0, 0,
               0, 0,-1, 0,
               0, 0, 0,-1;

    return diag_C4;
}

/**
* Sets, by new_base parameter, the pose of current base of a robotic system DQ_kinematics object and returns it in a constant DQ object.
*/
DQ const set_base(DQ_kinematics param_dq_kin, DQ new_base)
{
    return param_dq_kin.set_base(new_base);
}


/**
* Sets, by new_effector parameter, the pose of current end effector of a robotic system DQ_kinematics object and returns it in a constant DQ object.
*/
DQ const set_effector(DQ_kinematics param_dq_kin, DQ new_effector)
{
    return param_dq_kin.set_effector(new_effector);
}

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the last joint.
*/
DQ const raw_fkm(DQ_kinematics param_dq_kin, VectorXd theta_vec)
{
    return param_dq_kin.raw_fkm(theta_vec);
}

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the ith joint.
*/
DQ const raw_fkm(DQ_kinematics param_dq_kin, VectorXd theta_vec, int ith)
{
    return param_dq_kin.raw_fkm(theta_vec, ith);
}


/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the last joint
*/
DQ const fkm(DQ_kinematics param_dq_kin, VectorXd theta_vec)
{
    return param_dq_kin.fkm(theta_vec);
}

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the ith joint.

*/
DQ const fkm(DQ_kinematics param_dq_kin, VectorXd theta_vec, int ith)
{
    return param_dq_kin.fkm(theta_vec, ith);
}


/** Returns the correspondent DQ object, for a given link's Denavit Hartenberg parameters.
*/
DQ const dh2dq(DQ_kinematics param_dq_kin, double theta_ang, int link_i)
{
    return param_dq_kin.dh2dq(theta_ang, link_i);
}

DQ const get_z(DQ_kinematics param_dq_kin, VectorXd q)
{
    return param_dq_kin.get_z(q);
}

/** 
Returns a MatrixXd 8x(links - n_dummy) representing the Jacobian of a robotic system DQ_kinematics object.
*/
MatrixXd const jacobian(DQ_kinematics param_dq_kin, VectorXd theta_vec)
{
    return param_dq_kin.jacobian(theta_vec);
}

/** 
Returns a MatrixXd 8x(links - n_dummy) representing the Jacobian of a robotic system DQ_kinematics object.
*/
MatrixXd const analyticalJacobian(DQ_kinematics param_dq_kin, VectorXd theta_vec)
{
    return param_dq_kin.jacobian(theta_vec);
}


/**
* Obtains the rotation jacobian, relating the derivative of the rotation quaternion to the derivative of the joint variables as being the first 4 rows of the analytical jacobian.
* \param MatrixXd analytical_jacobian The robot analytical jacobian.
* \return The rotation jacobian.
*/
MatrixXd const rotationJacobian(MatrixXd analytical_jacobian)
{
	return analytical_jacobian.block(0,0,4,analytical_jacobian.cols());
}

/** Returns the translation Jacobian; that it, the Jacobian that satisfies the relation dot_p = Jp * dot_theta.
* Where dot_p is the time derivative of the translation quaternion and dot_theta is the time derivative of the joint vector.
* To use this member function type: 'dq_kin_object.jacobp(param_jacobian, x).
* \param Eigen::MatrixXd param_jacobian is the Jacobian of a robotic system
* \param Eigen::Matrix<double,8,1> x is the vector which constructs a translation DQ object
* \return A constant Eigen::MatrixXd
*/
MatrixXd const jacobp(MatrixXd analytical_jacobian, Matrix<double,8,1> x)
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

MatrixXd const translationJacobian(MatrixXd analytical_jacobian, Matrix<double,8,1> x)
{
    return DQ_robotics::jacobp(analytical_jacobian,x);
}

/** Returns the distance Jacobian; that it, the Jacobian that satisfies the relation dot_(d^2) = Jd * dot_theta.
* where dot_(d^2) is the time derivative of the square of the distance between the end-effector and the base and dot_theta is
* the time derivative of the joint vector.
* To use this member function type: 'dq_kin_object.jacobd(param_jacobian, x).
* \param Eigen::MatrixXd param_jacobian is the Jacobian of a robotic system
* \param Eigen::Matrix<double,8,1> x is the vector which constructs a translation DQ object
* \return A constant Eigen::MatrixXd
*/
MatrixXd const jacobd(MatrixXd param_jacobian, Matrix<double,8,1> x)
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

MatrixXd const distanceJacobian(MatrixXd param_jacobian, Matrix<double,8,1> x)
{
    return DQ_robotics::jacobd(param_jacobian,x);
}

/**
* Pseudo inverse implementation mimicking the on in MATLAB.
* \param Eigen::MatrixXd matrix: the matrix whose pseudoinverse you want.
* \return A constant Eigen::MatrixXd that is the Moore-Penrose pseudoinverse of matrix with the default MATLAB tolerance.
* \see  http://www.mathworks.com/help/matlab/ref/pinv.html
*/
MatrixXd const pseudoInverse(MatrixXd matrix)
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
* 'd' parameter 1 to n, 'a' parameter 1 to n, 'alpha' 1 to n and 'dummy joints' 1 to n parameters. The DH convention used is standard.
* To create a DQ_kinematics object using this, type: 'DQ dq_kin_object(A)';

* \param MatrixXd A contain the Denavit-Hartenberg parameters for the kinematic model.
*/
DQ_kinematics::DQ_kinematics(MatrixXd A) {
	
	if (A.rows() != 4 && A.rows() != 5) {
		std::cerr << "ERROR: DH Parameters matrix must be 4xn or 5xn. \n";
		system("PAUSE");
        }

        dq_kin.resize(A.rows(), A.cols());

        for(int i = 0; i < A.rows(); i++) {
            for(int j = 0; j < A.cols(); j++) {
                dq_kin(i,j) = A(i,j);
            }
        }

        aux_type = "standard";
        curr_base = DQ(1);
        curr_effector = DQ(1);
        
};

/**
* DQ_kinematics constructor using boost matrix
*
* Returns a DQ_kinematics object based on a 4xn or 5xn matrix named 'A' containing the Denavit-Hartenberg parameters.
* 'n' is the number of links of the robotic system. If 'A' has 4 rows there are no dummy joints. If 'A' has 4 rows,
* exists at least one dummy joint. 'A' matrix must be 4xn or 5xn. The 'A' rows 1 to 5 are respectively, 'theta' 1 to n,
* 'd' 1 to n, 'a' 1 to n, 'alpha' 1 to n and 'dummy joints' 1 to n parameters. The DH convention used is according to
* the 'type' parameter. 'type' is a string that can be 'standard' or 'modified' depending on the wanted convention. If
* something different of these values are attributed to 'type' parameter the standard convention is used as default.
* To create a DQ_kinematics object using this, type: 'DQ dq_kin_object(A,type)';

* \param MatrixXd A contain the Denavit-Hartenberg parameters for the kinematic model.
* \param std::string type contain the convention used in Denavit_Hartenberg.
*/
DQ_kinematics::DQ_kinematics(MatrixXd A, std::string type) {
 
    if (type != "standard" && type != "modified") {
        std::cerr << "ERROR: DH convention must be standard or modified. Write it correctly \n";
        system("PAUSE");
    }
    if (A.rows() != 4 && A.rows() != 5) {
        std::cerr << "ERROR: DH Parameters matrix must be 4xn or 5xn. \n";
        system("PAUSE");
    }

    dq_kin.resize(A.rows(), A.cols());
    for(int i = 0; i < A.rows(); i++) {
        for(int j = 0; j < A.cols(); j++) {
            dq_kin(i,j) = A(i,j);
        }
    }

    curr_base = DQ(1);
    curr_effector = DQ(1);
    aux_type = type;
};

/**
* DQ_kinematics Destructor
*
* Deletes from memory the DQ_kinematics object caller. To use this destructor, type: 'dq_kin_object.~DQ_kinematics();'. Dont need parameters.
*/
DQ_kinematics::~DQ_kinematics(){};

// Public constant methods

/**
* Returns a constant int representing the number of links of a robotic system DQ_kinematics object.
* It gets the number of columns of matrix 'A', passed to constructor and stored in the private attributte dq_kin.
* To use this member function, type: 'dq_kin_object.links();'.
* \return A constant int.
*/
int const DQ_kinematics::links() {
    return dq_kin.cols();
};

/**
* Returns a constant vector representing the theta joint angles offset of a robotic system DQ_kinematics object.
* It gets the first row of matrix 'A', passed to constructor and stored in the private attributte dq_kin.
* To use this member function, type: 'dq_kin_object.theta();'.
* \return A constant Eigen::VectorXd (number of links).
*/
VectorXd const DQ_kinematics::theta() {
    VectorXd aux_theta(dq_kin.cols());
    for (int i = 0; i < dq_kin.cols(); i++) {
        aux_theta(i) = dq_kin(0,i);
    }
    return aux_theta;
};



/**
* Returns a constant vector representing each 'd' offset along previous z to the common normal of a robotic system DQ_kinematics object.
* It gets the second row of matrix 'A', passed to constructor and stored in the private attributte dq_kin.
* To use this member function, type: 'dq_kin_object.d();'.
* \return A constant Eigen::VectorXd (number of links).
*/
VectorXd const DQ_kinematics::d() {
    VectorXd aux_d(dq_kin.cols());
    for (int i = 0; i < dq_kin.cols(); i++) {
        aux_d(i) = dq_kin(1,i);
    }
    return aux_d;
};

/**
* Returns a constant vector representing each 'a' length of the common normal of a robotic system DQ_kinematics object.
* It gets the third row of matrix 'A', passed to constructor and stored in the private attributte dq_kin.
* To use this member function, type: 'dq_kin_object.a();'.
* \return A constant Eigen::VectorXd (number of links).
*/
VectorXd const DQ_kinematics::a() {
    VectorXd aux_a(dq_kin.cols());
    for (int i = 0; i < dq_kin.cols(); i++) {
        aux_a(i) = dq_kin(2,i);
    }
    return aux_a;
};

/**
* Returns a constant vector representing each 'alpha' angle about common normal, from old z axis to new z axis of a
* robotic system DQ_kinematics object. It gets the fourth row of matrix 'A', passed to constructor and stored in the private attributte
* dq_kin. To use this member function, type: 'dq_kin_object.alpha();'.
* \return A constant Eigen::VectorXd (number of links).
*/
VectorXd const DQ_kinematics::alpha() {
    VectorXd aux_alpha(dq_kin.cols());
    for (int i = 0; i < dq_kin.cols(); i++) {
        aux_alpha(i) = dq_kin(3,i);
    }
    return aux_alpha;
};

/**
* Returns a constant vector representing the existing 'dummy' axes of a robotic system DQ_kinematics object.
* It gets the fifth row of matrix 'A', passed to constructor and stored in the private attributte dq_kin when it exists.
* If not, dummy vector is a null with n elements. Being n equal to number of links. To use this member function, type:
* 'dq_kin_object.dummy();'.
* \return A constant Eigen::VectorXd (number of links).
*/
VectorXd const DQ_kinematics::dummy() {
    VectorXd aux_dummy(dq_kin.cols());
    if (dq_kin.rows() > 4){
        for (int i = 0; i < dq_kin.cols(); i++) {
            aux_dummy(i) = dq_kin(4,i);
        }
        return aux_dummy;
    }
    else {
        for (int i = 0; i < dq_kin.cols(); i++) {
        aux_dummy(i) = 0;
        }
        return aux_dummy;
    }
};

/**
* Returns a constant int representing the number of 'dummy' axes of a robotic system DQ_kinematics object.
* If there are no dummy axes the result is 0. To use this member function, type: 'dq_kin_object.n_dummy();'.
* \return A constant int.
*/
int const DQ_kinematics::n_dummy() {
    int aux_n_dummy = 0;
    if (dq_kin.rows() > 4){
        for (int i = 0; i < dq_kin.cols(); i++) {
            if(dq_kin(4,i) == 1)
                aux_n_dummy = aux_n_dummy + 1;
        }
        return aux_n_dummy;
    }
    else
        return aux_n_dummy;
};

/**
* Returns a constant std::string representing the Denavit Hartenberg convenction (standard or modified) used in a robotic system
* DQ_kinematics object. To use this member function, type: 'dq_kin_object.convention();'.
* \return A constant std::string.
*/
std::string const DQ_kinematics::convention() {
    return aux_type;
};

/**
* Returns a constant DQ object representing current defined base of a robotic system DQ_kinematics object.
* To use this member function, type: 'dq_kin_object.base();'.
* \return A constant DQ object.
*/
DQ const DQ_kinematics::base() {
    return curr_base;
};

/**
* Returns a constant DQ object representing current defined end effector of a robotic system DQ_kinematics object.
* To use this member function, type: 'dq_kin_object.effector();'.
* \return A constant DQ object.
*/
DQ const DQ_kinematics::effector() {
    return curr_effector;
};

/**
* Sets, by new_base parameter, the pose of current base of a robotic system DQ_kinematics object and returns it in a constant DQ object.
* To use this member function, type: 'dq_kin_object.set_base();'.
* \param DQ new_base representing the new pose of robotic system base
* \return A constant DQ object.
*/
DQ const DQ_kinematics::set_base(DQ new_base) {
    curr_base = new_base;
    return curr_base;
};

/**
* Sets, by new_effector parameter, the pose of current end effector of a robotic system DQ_kinematics object and returns it in a constant DQ object.
* To use this member function, type: 'dq_kin_object.set_effector();'.
* \param DQ new_effector representing the new pose of robotic system end effector
* \return A constant DQ object.
*/
DQ const DQ_kinematics::set_effector(DQ new_effector) {
    curr_effector = new_effector;
    return curr_effector;
};

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the last joint.
* The displacements due to the base and the effector are not taken into account. theta_vec is the vector of joint variables.
* This is an auxiliary function to be used mainly with the jacobian function.
* To use this member function, type: 'dq_kin_object.raw_fkm(theta_vec);'.
* \param Eigen::VectorXd theta_vec is the vector representing the theta joint angles.
* \return A constant DQ object.
*/
DQ const DQ_kinematics::raw_fkm(VectorXd theta_vec) {
    DQ_kinematics aux_dq_kin(dq_kin, aux_type);
    if((int)theta_vec.size() != (aux_dq_kin.links() - aux_dq_kin.n_dummy()) ) {
        //erro
        cout << "\n INCORRECT NUMBER OF JOINT VARIABLES \n";
    }
    DQ q(1);
    int j = 0;
    for (int i = 0; i < aux_dq_kin.links(); i++) {
        if(aux_dq_kin.dummy()(i) == 1) {
            q = q * DQ_robotics::dh2dq(aux_dq_kin, 0, i+1);
            j = j + 1;
        }
        else
            q = q * DQ_robotics::dh2dq(aux_dq_kin, theta_vec(i-j), i+1);
    }
    return q;
};

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the ith joint.
* The displacements due to the base and the effector are not taken into account. theta_vec is the vector of joint variables.
* This is an auxiliary function to be used mainly with the jacobian function.
* To use this member function, type: 'dq_kin_object.raw_fkm(theta_vec, ith);'.
* \param Eigen::VectorXd theta_vec is the vector representing the theta joint angles.
* \param int ith is the position of the least joint included in the forward kinematic model
* \return A constant DQ object.
*/
DQ const DQ_kinematics::raw_fkm(VectorXd theta_vec, int ith) {
    DQ_kinematics aux_dq_kin(dq_kin, aux_type);
    if((int)theta_vec.size() != (aux_dq_kin.links() - aux_dq_kin.n_dummy()) ) {
        //erro
        cout << "\n INCORRECT NUMBER OF JOINT VARIABLES \n";
    }
    DQ q(1);
    int j = 0;
    for (int i = 0; i < ith; i++) {
        if(aux_dq_kin.dummy()(i) == 1) {
            q = q * DQ_robotics::dh2dq(aux_dq_kin, 0, i+1);
            j = j + 1;
        }
        else
            q = q * DQ_robotics::dh2dq(aux_dq_kin, theta_vec(i-j), i+1);
    }
    return q;
};

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the last joint.
* This function takes into account the displacement due to the base's and effector's poses. theta_vec is the vector of joint variables.
* To use this member function, type: 'dq_kin_object.fkm(theta_vec);'.
* \param Eigen::VectorXd theta_vec is the vector representing the theta joint angles.
* \return A constant DQ object.
*/
DQ const DQ_kinematics::fkm(VectorXd theta_vec) {
    DQ_kinematics aux_dq_kin(dq_kin, aux_type);
    DQ q = curr_base * aux_dq_kin.raw_fkm(theta_vec) * curr_effector;
    return q;
};


/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the ith joint.
* This function takes into account the displacement due to the base's and effector's poses. theta_vec is the vector of joint variables.
* To use this member function, type: 'dq_kin_object.fkm(theta_vec, ith);'.
* \param Eigen::VectorXd theta_vec is the vector representing the theta joint angles.
* \return A constant DQ object.
*/
DQ const DQ_kinematics::fkm(VectorXd theta_vec, int ith) {
    DQ_kinematics aux_dq_kin(dq_kin, aux_type);
    DQ q = curr_base * aux_dq_kin.raw_fkm(theta_vec, ith) * curr_effector;
    return q;
};

/** Returns the correspondent DQ object, for a given link's Denavit Hartenberg parameters.
* To use this member function type: 'dq_kin_object.dh2dq(theta,i), where theta is the joint angle and i is the link number
* \param double theta_ang is the joint angle
* \param int link_i is the link number
* \return A constant DQ object
*/
DQ const DQ_kinematics::dh2dq(double theta_ang, int link_i) {
    DQ_kinematics aux_dq_kin(dq_kin, aux_type);
    Matrix<double,8,1> q(8);
    link_i = link_i - 1;

    double d = aux_dq_kin.d()(link_i);
    double a = aux_dq_kin.a()(link_i);
    double alpha = aux_dq_kin.alpha()(link_i);
    //std::string standard = "standard";

    if(aux_dq_kin.convention() == "standard") {

        q(0)=cos((theta_ang + aux_dq_kin.theta()(link_i) )/2)*cos(alpha/2);
        q(1)=cos((theta_ang + aux_dq_kin.theta()(link_i) )/2)*sin(alpha/2);
        q(2)=sin((theta_ang + aux_dq_kin.theta()(link_i) )/2)*sin(alpha/2);
        q(3)=sin((theta_ang + aux_dq_kin.theta()(link_i) )/2)*cos(alpha/2);
        double d2=d/2;
        double a2=a/2;
        q(4)= -d2*q(3) - a2*q(1);
        q(5)= -d2*q(2) + a2*q(0);
        q(6)= d2*q(1) + a2*q(3);
        q(7)= d2*q(0) - a2*q(2);
    }
    else{

        double h1 = cos((theta_ang + aux_dq_kin.theta()(link_i) )/2)*cos(alpha/2);
        double h2 = cos((theta_ang + aux_dq_kin.theta()(link_i) )/2)*sin(alpha/2);
        double h3 = sin((theta_ang + aux_dq_kin.theta()(link_i) )/2)*sin(alpha/2);
        double h4 = sin((theta_ang + aux_dq_kin.theta()(link_i) )/2)*cos(alpha/2);
        q(0)= h1;
        q(1)= h2;
        q(2)= -h3;
        q(3)= h4;
        double d2=d/2;
        double a2=a/2;
        q(4)=-d2*h4 - a2*h2;
        q(5)=-d2*h3 + a2*h1;
        q(6)=-(d2*h2 + a2*h4);
        q(7)=d2*h1 - a2*h3;
    }
    return DQ(q);
};


DQ const DQ_kinematics::get_z(VectorXd q) {
    Matrix<double,8,1> z(8);
    z(0) = 0;
    z(1)=q(1)*q(3) + q(0)*q(2);
    z(2)=q(2)*q(3) - q(0)* q(1);
    z(3)=(q(3)*q(3)-q(2)*q(2)-q(1)*q(1)+q(0)*q(0))/2;
    z(4)=0;
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
MatrixXd const DQ_kinematics::jacobian(VectorXd theta_vec) {
    DQ_kinematics aux_dq_kin(dq_kin, aux_type);
    DQ q_effector = aux_dq_kin.raw_fkm(theta_vec);

    DQ z;
    DQ q(1);

    MatrixXd J(8,(aux_dq_kin.links() - aux_dq_kin.n_dummy()) );

    for (int i = 0; i < J.rows(); i++) {
        for(int j = 0; j < J.cols(); j++) {
            J(i,j) = 0;
        }
    }
    int ith = -1;
    for(int i = 0; i < aux_dq_kin.links(); i++) {

            // Use the standard DH convention
            if(aux_dq_kin.convention() == "standard") {
                z = aux_dq_kin.get_z(q.q);
            }
            // Use the modified DH convention
            else {
                DQ w(0, 0, -sin(aux_dq_kin.alpha()(i)), cos(aux_dq_kin.alpha()(i)), 0, 0, -aux_dq_kin.a()(i)*cos(aux_dq_kin.alpha()(i)), -aux_dq_kin.a()(i)*sin(aux_dq_kin.alpha()(i)));
                z =0.5 * q * w * q.conj();
            }

            if(aux_dq_kin.dummy()(i) == 0) {
            	q = q * aux_dq_kin.dh2dq(theta_vec(ith+1), i+1);
            	DQ aux_j = z * q_effector;
            	for(int k = 0; k < J.rows(); k++) {
            	    J(k,ith+1) = aux_j.q(k);
            	}
                ith = ith+1;
            }
	    else
		// Dummy joints don't contribute to the Jacobian
            q = q * aux_dq_kin.dh2dq(0,(i+1));
    }
    // Takes the base's displacement into account
    Matrix<double,8,8> aux_J(8,8);
    aux_J = Hminus8(curr_effector);
    aux_J = Hplus8(curr_base)*aux_J;
    J = aux_J*J;
    return J;
};

MatrixXd const DQ_kinematics::analyticalJacobian(VectorXd theta_vec)
{
    return DQ_kinematics::jacobian(theta_vec);
}




}//namespace DQ_robotics
