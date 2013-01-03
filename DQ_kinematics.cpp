#include "DQ_kinematics.h"
#include "DQ.h"
#include <iostream>
#include <iomanip>
#include<math.h>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace boost::numeric::ublas;
using std::cout;

/**
* DQ_kinematics constructor using boost matrix
*
* Returns a DQ_kinematics object based on a 4xn or 5xn matrix named 'A' containing the Denavit-Hartenberg parameters.
* 'n' is the number of links of the robotic system. If 'A' has 4 rows there are no dummy joints. If 'A' has 4 rows,
* exists at least one dummy joint. 'A' matrix must be 4xn or 5xn. The 'A' rows 1 to 5 are respectively, 'theta' 1 to n,
* 'd' parameter 1 to n, 'a' parameter 1 to n, 'alpha' 1 to n and 'dummy joints' 1 to n parameters. The DH convention used is standard.
* To create a DQ_kinematics object using this, type: 'DQ dq_kin_object(A)';

* \param matrix <double> A contain the Denavit-Hartenberg parameters for the kinematic model.
*/
DQ_kinematics::DQ_kinematics(matrix <double> A) {
    try {
        if (A.size1() != 4 && A.size1() != 5) {
            throw 1;
        }

        dq_kin.resize(A.size1(), A.size2());
        for(unsigned int i = 0; i < A.size1(); i++) {
            for(unsigned int j = 0; j < A.size2(); j++) {
                dq_kin(i,j) = A(i,j);
            }
        }
        aux_type = "standard";
        curr_base = DQ(1);
        curr_effector = DQ(1);
        }

    catch (int i) {
        std::cerr << "ERROR: DH Parameters matrix must be 4xn or 5xn. \n";
        system("PAUSE");
        }
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

* \param matrix <double> A contain the Denavit-Hartenberg parameters for the kinematic model.
* \param std::string type contain the convention used in Denavit_Hartenberg.
*/
DQ_kinematics::DQ_kinematics(matrix <double> A, std::string type) {
    curr_base = DQ(1);
    curr_effector = DQ(1);

    if (A.size1() != 4 && A.size1() != 5) {
        std::cerr << "ERROR: DH Parameters matrix must be 4xn or 5xn. \n";
        system("PAUSE");
    }
    dq_kin.resize(A.size1(), A.size2());
    for(unsigned int i = 0; i < A.size1(); i++) {
        for(unsigned int j = 0; j < A.size2(); j++) {
            dq_kin(i,j) = A(i,j);
        }
    }
    if (type != "standard" && type != "modified") {
        std::cerr << "ERROR: DH convention must be standard or modified. Write it correctly \n";
        system("PAUSE");
    }
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
    return dq_kin.size2();
};

/**
* Returns a constant int representing the number of links of a robotic system DQ_kinematics object.
* Actually this function does the same as links() changing only the way of calling, which is DQ::links(dq_kin_object).
*/
int const DQ_kinematics::links(DQ_kinematics param_dq_kin) {
    return param_dq_kin.links();
};

/**
* Returns a constant vector representing the theta joint angles offset of a robotic system DQ_kinematics object.
* It gets the first row of matrix 'A', passed to constructor and stored in the private attributte dq_kin.
* To use this member function, type: 'dq_kin_object.theta();'.
* \return A constant boost::numeric::ublas::vector <double> (number of links).
*/
vector <double> const DQ_kinematics::theta() {
    vector <double> aux_theta(dq_kin.size2());
    for (unsigned int i = 0; i < dq_kin.size2(); i++) {
        aux_theta(i) = dq_kin(0,i);
    }
    return aux_theta;
};

/**
* Returns a constant vector representing the theta joint angles offset of a robotic system DQ_kinematics object.
* Actually this function does the same as theta() changing only the way of calling, which is DQ::theta(dq_kin_object).
*/
vector <double> const DQ_kinematics::theta(DQ_kinematics param_dq_kin) {
    return param_dq_kin.theta();
};

/**
* Returns a constant vector representing each 'd' offset along previous z to the common normal of a robotic system DQ_kinematics object.
* It gets the second row of matrix 'A', passed to constructor and stored in the private attributte dq_kin.
* To use this member function, type: 'dq_kin_object.d();'.
* \return A constant boost::numeric::ublas::vector <double> (number of links).
*/
vector <double> const DQ_kinematics::d() {
    vector <double> aux_d(dq_kin.size2());
    for (unsigned int i = 0; i < dq_kin.size2(); i++) {
        aux_d(i) = dq_kin(1,i);
    }
    return aux_d;
};

/**
* Returns a constant vector representing each 'd' offset along previous z to the common normal of a robotic system DQ_kinematics object.
* Actually this function does the same as d() changing only the way of calling, which is DQ::d(dq_kin_object).
*/
vector <double> const DQ_kinematics::d(DQ_kinematics param_dq_kin) {
    return param_dq_kin.d();
};

/**
* Returns a constant vector representing each 'a' length of the common normal of a robotic system DQ_kinematics object.
* It gets the third row of matrix 'A', passed to constructor and stored in the private attributte dq_kin.
* To use this member function, type: 'dq_kin_object.a();'.
* \return A constant boost::numeric::ublas::vector <double> (number of links).
*/
vector <double> const DQ_kinematics::a() {
    vector <double> aux_a(dq_kin.size2());
    for (unsigned int i = 0; i < dq_kin.size2(); i++) {
        aux_a(i) = dq_kin(2,i);
    }
    return aux_a;
};

/**
* Returns a constant vector representing each 'a' length of the common normal of a robotic system DQ_kinematics object.
* Actually this function does the same as a() changing only the way of calling, which is DQ::a(dq_kin_object).
*/
vector <double> const DQ_kinematics::a(DQ_kinematics param_dq_kin) {
    return param_dq_kin.a();
};

/**
* Returns a constant vector representing each 'alpha' angle about common normal, from old z axis to new z axis of a
* robotic system DQ_kinematics object. It gets the fourth row of matrix 'A', passed to constructor and stored in the private attributte
* dq_kin. To use this member function, type: 'dq_kin_object.alpha();'.
* \return A constant boost::numeric::ublas::vector <double> (number of links).
*/
vector <double> const DQ_kinematics::alpha() {
    vector <double> aux_alpha(dq_kin.size2());
    for (unsigned int i = 0; i < dq_kin.size2(); i++) {
        aux_alpha(i) = dq_kin(3,i);
    }
    return aux_alpha;
};

/**
* Returns a constant vector representing each 'alpha' angle about common normal, from old z axis to new z axis of a
* robotic system DQ_kinematics object. Actually this function does the same as alpha() changing only the way of calling,
* which is DQ::alpha(dq_kin_object).
*/
vector <double> const DQ_kinematics::alpha(DQ_kinematics param_dq_kin) {
    return param_dq_kin.alpha();
};

/**
* Returns a constant vector representing the existing 'dummy' axes of a robotic system DQ_kinematics object.
* It gets the fifth row of matrix 'A', passed to constructor and stored in the private attributte dq_kin when it exists.
* If not, dummy vector is a null with n elements. Being n equal to number of links. To use this member function, type:
* 'dq_kin_object.dummy();'.
* \return A constant boost::numeric::ublas::vector <double> (number of links).
*/
vector <double> const DQ_kinematics::dummy() {
    vector <double> aux_dummy(dq_kin.size2());
    if (dq_kin.size1() > 4){
        for (unsigned int i = 0; i < dq_kin.size2(); i++) {
            aux_dummy(i) = dq_kin(4,i);
        }
        return aux_dummy;
    }
    else {
        for (unsigned int i = 0; i < dq_kin.size2(); i++) {
        aux_dummy(i) = 0;
        }
        return aux_dummy;
    }
};

/**
* Returns a constant vector representing the existing 'dummy' axes of a robotic system DQ_kinematics object.
* Actually this function does the same as dummy() changing only the way of calling, which is DQ::dummy(dq_kin_object).
*/
vector <double> const DQ_kinematics::dummy(DQ_kinematics param_dq_kin) {
    return param_dq_kin.dummy();
};

/**
* Returns a constant int representing the number of 'dummy' axes of a robotic system DQ_kinematics object.
* If there are no dummy axes the result is 0. To use this member function, type: 'dq_kin_object.n_dummy();'.
* \return A constant int.
*/
int const DQ_kinematics::n_dummy() {
    int aux_n_dummy = 0;
    if (dq_kin.size1() > 4){
        for (unsigned int i = 0; i < dq_kin.size2(); i++) {
            if(dq_kin(4,i) == 1)
                aux_n_dummy = aux_n_dummy + 1;
        }
        return aux_n_dummy;
    }
    else
        return aux_n_dummy;
};

/**
* Returns a constant int representing the number of 'dummy' axes of a robotic system DQ_kinematics object.
* Actually this function does the same as n_dummy() changing only the way of calling, which is DQ::n_dummy(dq_kin_object).
*/
int const DQ_kinematics::n_dummy(DQ_kinematics param_dq_kin) {
    return param_dq_kin.n_dummy();
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
* Returns a constant std::string representing the Denavit Hartenberg convenction (standard or modified) used in a robotic system
* DQ_kinematics object. Actually this function does the same as convention() changing only the way of calling, which is
* DQ::convention(dq_kin_object).
*/
std::string const DQ_kinematics::convention(DQ_kinematics param_dq_kin) {
    return param_dq_kin.convention();
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
* Returns a constant DQ object representing current defined base of a robotic system DQ_kinematics object.
* Actually this function does the same as base() changing only the way of calling, which is DQ::base(dq_kin_object).
*/
DQ const DQ_kinematics::base(DQ_kinematics param_dq_kin) {
    return param_dq_kin.base();
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
* Returns a constant DQ object representing current defined end effector of a robotic system DQ_kinematics object.
* Actually this function does the same as effector() changing only the way of calling, which is DQ::effector(dq_kin_object).
*/
DQ const DQ_kinematics::effector(DQ_kinematics param_dq_kin) {
    return param_dq_kin.effector();
};

/**
* Returns a constant matrix <double> 8x8 dimensions representing C8, a diagonal negative unit matrix.
* Given the jacobian matrix J that satisfies 'vec8(dot_x) = J * dot_theta', where dot_x is the time derivative of the translation
* quaternion and dot_theta is the time derivative of the joint vector, the following relation
* is established: 'vec8(dot_x) = C8 * J * dot_theta'. To use this member function, type: 'dq_kin_object.C8();'.
* \return A constant boost::numeric::ublas::matrix <double> (8,8).
*/
matrix <double> const DQ_kinematics::C8() {
	matrix <double> diag_C8(8,8);
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
};

/**
* Returns a constant matrix <double> 8x8 dimensions representing C8, a diagonal negative unit matrix.
* Actually this function does the same as C8() changing only the way of calling, which is DQ::C8(dq_kin_object).
*/
matrix <double> const DQ_kinematics::C8(DQ_kinematics param_dq_kin) {
    return param_dq_kin.C8();
};

/**
* Returns a constant matrix <double> 4x4 dimensions representing C4, a diagonal negative unit matrix.
* Given the jacobian matrix J that satisfies 'vec4(dot_x) = J * dot_theta', where dot_x is the time derivative of the translation
* quaternion and dot_theta is the time derivative of the joint vector, the following relation
* is established: 'vec4(dot_x) = C4 * J * dot_theta'. To use this member function, type: 'dq_kin_object.C4();'.
* \return A constant boost::numeric::ublas::matrix <double> (4,4).
*/
matrix <double> const DQ_kinematics::C4() {
	matrix <double> diag_C4(4,4);
    diag_C4(0,0) = 1; diag_C4(0,1) = 0; diag_C4(0,2) = 0; diag_C4(0,3) = 0;
    diag_C4(1,0) = 0; diag_C4(1,1) = -1; diag_C4(1,2) = 0; diag_C4(1,3) = 0;
    diag_C4(2,0) = 0; diag_C4(2,1) = 0; diag_C4(2,2) = -1; diag_C4(2,3) = 0;
    diag_C4(3,0) = 0; diag_C4(3,1) = 0; diag_C4(3,2) = 0; diag_C4(3,3) = -1;
    return diag_C4;
};

/**
* Returns a constant matrix <double> 4x4 dimensions representing C4, a diagonal negative unit matrix.
* Actually this function does the same as C4() changing only the way of calling, which is DQ::C4(dq_kin_object).
*/
matrix <double> const DQ_kinematics::C4(DQ_kinematics param_dq_kin) {
    return param_dq_kin.C4();
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
* Sets, by new_base parameter, the pose of current base of a robotic system DQ_kinematics object and returns it in a constant DQ object.
* Actually this function does the same as set_base() changing only the way of calling, which is DQ::set_base(dq_kin_object).
*/
DQ const DQ_kinematics::set_base(DQ_kinematics param_dq_kin, DQ new_base) {
    return param_dq_kin.set_base(new_base);
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
* Sets, by new_effector parameter, the pose of current end effector of a robotic system DQ_kinematics object and returns it in a constant DQ object.
* Actually this function does the same as set_effector() changing only the way of calling, which is DQ::set_effector(dq_kin_object).
*/
DQ const DQ_kinematics::set_effector(DQ_kinematics param_dq_kin, DQ new_effector) {
    return param_dq_kin.set_effector(new_effector);
};

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the last joint.
* The displacements due to the base and the effector are not taken into account. theta_vec is the vector of joint variables.
* This is an auxiliary function to be used mainly with the jacobian function.
* To use this member function, type: 'dq_kin_object.raw_fkm(theta_vec);'.
* \param boost::numeric::ublas::vector <double> theta_vec is the vector representing the theta joint angles.
* \return A constant DQ object.
*/
DQ const DQ_kinematics::raw_fkm(vector <double> theta_vec) {
    DQ_kinematics aux_dq_kin(dq_kin, aux_type);
    if((int)theta_vec.size() != (aux_dq_kin.links() - aux_dq_kin.n_dummy()) ) {
        //erro
        cout << "\n INCORRECT NUMBER OF JOINT VARIABLES \n";
    }
    DQ q(1);
    int j = 0;
    for (int i = 0; i < aux_dq_kin.links(); i++) {
        if(aux_dq_kin.dummy()(i) == 1) {
            q = q * dh2dq(aux_dq_kin, 0, i+1);
            j = j + 1;
        }
        else
            q = q * dh2dq(aux_dq_kin, theta_vec(i-j), i+1);
    }
    return q;
};

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the last joint.
* Actually this function does the same as raw_fkm(theta_vec) changing only the way of calling, which is
* DQ::raw_fkm(dq_kin_object, theta_vec).
*/
DQ const DQ_kinematics::raw_fkm(DQ_kinematics param_dq_kin, vector <double> theta_vec) {
    return param_dq_kin.raw_fkm(theta_vec);
};

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the ith joint.
* The displacements due to the base and the effector are not taken into account. theta_vec is the vector of joint variables.
* This is an auxiliary function to be used mainly with the jacobian function.
* To use this member function, type: 'dq_kin_object.raw_fkm(theta_vec, ith);'.
* \param boost::numeric::ublas::vector <double> theta_vec is the vector representing the theta joint angles.
* \param int ith is the position of the least joint included in the forward kinematic model
* \return A constant DQ object.
*/
DQ const DQ_kinematics::raw_fkm(vector <double> theta_vec, int ith) {
    DQ_kinematics aux_dq_kin(dq_kin, aux_type);
    if((int)theta_vec.size() != (aux_dq_kin.links() - aux_dq_kin.n_dummy()) ) {
        //erro
        cout << "\n INCORRECT NUMBER OF JOINT VARIABLES \n";
    }
    DQ q(1);
    int j = 0;
    for (int i = 0; i < ith; i++) {
        if(aux_dq_kin.dummy()(i) == 1) {
            q = q * dh2dq(aux_dq_kin, 0, i+1);
            j = j + 1;
        }
        else
            q = q * dh2dq(aux_dq_kin, theta_vec(i-j), i+1);
    }
    return q;
};

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the ith joint.
* Actually this function does the same as raw_fkm(theta_vec, ith) changing only the way of calling, which is
* DQ::raw_fkm(dq_kin_object, theta_vec, ith).
*/
DQ const DQ_kinematics::raw_fkm(DQ_kinematics param_dq_kin, vector <double> theta_vec, int ith) {
    return param_dq_kin.raw_fkm(theta_vec, ith);
};

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the last joint.
* This function takes into account the displacement due to the base's and effector's poses. theta_vec is the vector of joint variables.
* To use this member function, type: 'dq_kin_object.fkm(theta_vec);'.
* \param boost::numeric::ublas::vector <double> theta_vec is the vector representing the theta joint angles.
* \return A constant DQ object.
*/
DQ const DQ_kinematics::fkm(vector <double> theta_vec) {
    DQ_kinematics aux_dq_kin(dq_kin, aux_type);
    DQ q = aux_dq_kin.base() * aux_dq_kin.raw_fkm(theta_vec) * aux_dq_kin.effector();
    return q;
};

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the last joint.
* Actually this function does the same as fkm(theta_vec) changing only the way of calling, which is
* DQ::fkm(dq_kin_object, theta_vec).
*/
DQ const DQ_kinematics::fkm(DQ_kinematics param_dq_kin, vector <double> theta_vec) {
    return param_dq_kin.fkm(theta_vec);
};

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the ith joint.
* This function takes into account the displacement due to the base's and effector's poses. theta_vec is the vector of joint variables.
* To use this member function, type: 'dq_kin_object.fkm(theta_vec, ith);'.
* \param boost::numeric::ublas::vector <double> theta_vec is the vector representing the theta joint angles.
* \return A constant DQ object.
*/
DQ const DQ_kinematics::fkm(vector <double> theta_vec, int ith) {
    DQ_kinematics aux_dq_kin(dq_kin, aux_type);
    DQ q = aux_dq_kin.base() * aux_dq_kin.raw_fkm(theta_vec, ith) * aux_dq_kin.effector();
    return q;
};

/**
* Calculates the forward kinematic model and returns a DQ object corresponding to the ith joint.
* Actually this function does the same as fkm(theta_vec, ith) changing only the way of calling, which is
* DQ::fkm(dq_kin_object, theta_vec, ith).
*/
DQ const DQ_kinematics::fkm(DQ_kinematics param_dq_kin, vector <double> theta_vec, int ith) {
    return param_dq_kin.fkm(theta_vec, ith);
};

/** Returns the correspondent DQ object, for a given link's Denavit Hartenberg parameters.
* To use this member function type: 'dq_kin_object.dh2dq(theta,i), where theta is the joint angle and i is the link number
* \param double theta_ang is the joint angle
* \param int link_i is the link number
* \return A constant DQ object
*/
DQ const DQ_kinematics::dh2dq(double theta_ang, int link_i) {
    DQ_kinematics aux_dq_kin(dq_kin, aux_type);
    vector <double> q(8);
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

/** Returns the correspondent DQ object, for a given link's Denavit Hartenberg parameters.
* Actually this function does the same as dh2dq(theta_ang, link_i) changing only the way of calling, which is
* DQ::dh2dq(dq_kin_object, theta_ang, link_i).
*/
DQ const DQ_kinematics::dh2dq(DQ_kinematics param_dq_kin, double theta_ang, int link_i) {
    return param_dq_kin.dh2dq(theta_ang, link_i);
};


DQ const DQ_kinematics::get_z(vector <double> q) {
    vector <double> z(8);
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

DQ const DQ_kinematics::get_z(DQ_kinematics param_dq_kin, vector <double> q) {
    return param_dq_kin.get_z(q);
};

/** Returns a matrix <double> 8x(links - n_dummy) representing the Jacobian of a robotic system DQ_kinematics object.
* theta_vec is the vector of joint variables.
* \param boost::numeric::ublas::vector <double> theta_vec is the vector representing the theta joint angles.
* \return A constant boost::numeric::ublas::matrix <double> (8,links - n_dummy).
*/
matrix <double> const DQ_kinematics::jacobian(vector <double> theta_vec) {
    DQ_kinematics aux_dq_kin(dq_kin, aux_type);
    DQ q_effector = aux_dq_kin.raw_fkm(theta_vec);

    DQ z;
    DQ q(1);

    matrix <double> J(8,(aux_dq_kin.links() - aux_dq_kin.n_dummy()) );

    for (unsigned int i = 0; i < J.size1(); i++) {
        for(unsigned int j = 0; j < J.size2(); j++) {
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
            	for(unsigned int k = 0; k < J.size1(); k++) {
            	    J(k,ith+1) = aux_j.q(k);
            	}
                ith = ith+1;
            }
	    else
		// Dummy joints don't contribute to the Jacobian
            q = q * aux_dq_kin.dh2dq(0,(i+1));
    }
    // Takes the base's displacement into account
    matrix <double> aux_J(8,8);
    aux_J = DQ::Hminus8(aux_dq_kin.effector());
    aux_J = prod(DQ::Hplus8(aux_dq_kin.base()), aux_J);
    J = prod(aux_J, J);
    return J;
};

/** Returns a matrix <double> 8x(links - n_dummy) representing the Jacobian of a robotic system DQ_kinematics object.
* Actually this function does the same as jacobian(theta_vec) changing only the way of calling, which is
* DQ::jacobian(dq_kin_object, theta_vec).
*/
matrix <double> const DQ_kinematics::jacobian(DQ_kinematics param_dq_kin, vector <double> theta_vec) {
    return param_dq_kin.jacobian(theta_vec);
};

/** Returns the translation Jacobian; that it, the Jacobian that satisfies the relation dot_p = Jp * dot_theta.
* Where dot_p is the time derivative of the translation quaternion and dot_theta is the time derivative of the joint vector.
* To use this member function type: 'dq_kin_object.jacobp(param_jacobian, x).
* \param boost::numeric::ublas::matrix <double> param_jacobian is the Jacobian of a robotic system
* \param boost::numeric::ublas::vector <double> x is the vector which constructs a translation DQ object
* \return A constant boost::numeric::ublas::matrix <double>
*/
matrix <double> const DQ_kinematics::jacobp(matrix <double> param_jacobian, vector <double> x) {
    DQ dq_x(x);
    DQ dq_x_conj_P = dq_x.P();
    dq_x_conj_P = dq_x_conj_P.conj();
    matrix <double> aux_J1(4,param_jacobian.size2());
    matrix <double> aux_J2(4,param_jacobian.size2());
    for(int i = 0; i < 4; i++) {
        for(unsigned int j = 0; j < param_jacobian.size2(); j++) {
            aux_J1(i,j) = param_jacobian(i,j);
            aux_J2(i,j) = param_jacobian((i+4),j);
        }
    }
    matrix <double> aux = prod(DQ::Hplus4(dq_x.D()), DQ_kinematics::C4());
    matrix <double> Jp = 2*prod(DQ::Hminus4(dq_x_conj_P), aux_J2) + 2*prod(aux, aux_J1);
    return Jp;
};

/** Returns the translation Jacobian; that it, the Jacobian that satisfies the relation dot_p = Jp * dot_theta.
* Actually this function does the same as jacobp(param_jacobian, x) changing only the way of calling, which is
* DQ::jacobp(dq_kin_object, param_jacobian, x).
*/
matrix <double> const DQ_kinematics::jacobp(DQ_kinematics param_dq_kin, matrix <double> param_jacobian, vector <double> x) {
    return param_dq_kin.jacobp(param_jacobian, x);
};

/** Returns the distance Jacobian; that it, the Jacobian that satisfies the relation dot_(d^2) = Jd * dot_theta.
* where dot_(d^2) is the time derivative of the square of the distance between the end-effector and the base and dot_theta is
* the time derivative of the joint vector.
* To use this member function type: 'dq_kin_object.jacobd(param_jacobian, x).
* \param boost::numeric::ublas::matrix <double> param_jacobian is the Jacobian of a robotic system
* \param boost::numeric::ublas::vector <double> x is the vector which constructs a translation DQ object
* \return A constant boost::numeric::ublas::matrix <double>
*/
matrix <double> const DQ_kinematics::jacobd(matrix <double> param_jacobian, vector <double> x) {
    DQ dq_x(x);
    DQ p = DQ::translation(dq_x);
    matrix <double> Jp = DQ_kinematics::jacobp(param_jacobian, x);
    matrix <double> Jd = 2 * prod(DQ::vec4(p), Jp);
    return Jd;
};

/** Returns the distance Jacobian; that it, the Jacobian that satisfies the relation dot_(d^2) = Jd * dot_theta.
* Actually this function does the same as jacobd(param_jacobian, x) changing only the way of calling, which is
* DQ::jacobd(dq_kin_object, param_jacobian, x).
*/
matrix <double> const DQ_kinematics::jacobd(DQ_kinematics param_dq_kin, matrix <double> param_jacobian, vector <double> x) {
    return param_dq_kin.jacobd(param_jacobian, x);
};
