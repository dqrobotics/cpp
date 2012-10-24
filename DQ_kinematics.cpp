#include "DQ_kinematics.h"
#include "DQ.h"
#include <iostream>
#include <iomanip>
#include<math.h>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace boost::numeric::ublas;
using std::cout;

DQ_kinematics::DQ_kinematics(matrix <double> A) {
    dq_kin.resize(A.size1(), A.size2());
    for(unsigned int i = 0; i < A.size1(); i++) {
        for(unsigned int j = 0; j < A.size2(); j++) {
        dq_kin(i,j) = A(i,j);
        }
    }
    aux_type = "standard";
    curr_base = DQ(1);
};

DQ_kinematics::DQ_kinematics(matrix <double> A, std::string type) {
    dq_kin.resize(A.size1(), A.size2());
    for(unsigned int i = 0; i < A.size1(); i++) {
        for(unsigned int j = 0; j < A.size2(); j++) {
        dq_kin(i,j) = A(i,j);
        }
    }
    aux_type = type;
    curr_base = DQ(1);
};

DQ_kinematics::~DQ_kinematics(){};

int const DQ_kinematics::links() {
    return dq_kin.size2();
};

int const DQ_kinematics::links(DQ_kinematics param_dq_kin) {
    return param_dq_kin.links();
};

vector <double> const DQ_kinematics::theta() {
    vector <double> aux_theta(dq_kin.size2());
    for (unsigned int i = 0; i < dq_kin.size2(); i++) {
        aux_theta(i) = dq_kin(0,i);
    }
    return aux_theta;
};

vector <double> const DQ_kinematics::theta(DQ_kinematics param_dq_kin) {
    return param_dq_kin.theta();
};

vector <double> const DQ_kinematics::d() {
    vector <double> aux_d(dq_kin.size2());
    for (unsigned int i = 0; i < dq_kin.size2(); i++) {
        aux_d(i) = dq_kin(1,i);
    }
    return aux_d;
};

vector <double> const DQ_kinematics::d(DQ_kinematics param_dq_kin) {
    return param_dq_kin.d();
};

vector <double> const DQ_kinematics::a() {
    vector <double> aux_a(dq_kin.size2());
    for (unsigned int i = 0; i < dq_kin.size2(); i++) {
        aux_a(i) = dq_kin(2,i);
    }
    return aux_a;
};

vector <double> const DQ_kinematics::a(DQ_kinematics param_dq_kin) {
    return param_dq_kin.a();
};

vector <double> const DQ_kinematics::alpha() {
    vector <double> aux_alpha(dq_kin.size2());
    for (unsigned int i = 0; i < dq_kin.size2(); i++) {
        aux_alpha(i) = dq_kin(3,i);
    }
    return aux_alpha;
};

vector <double> const DQ_kinematics::alpha(DQ_kinematics param_dq_kin) {
    return param_dq_kin.alpha();
};

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

vector <double> const DQ_kinematics::dummy(DQ_kinematics param_dq_kin) {
    return param_dq_kin.dummy();
};

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

int const DQ_kinematics::n_dummy(DQ_kinematics param_dq_kin) {
    return param_dq_kin.n_dummy();
};

std::string const DQ_kinematics::convention() {
    return aux_type;
};

std::string const DQ_kinematics::convention(DQ_kinematics param_dq_kin) {
    return param_dq_kin.convention();
};

DQ const DQ_kinematics::base() {
    return curr_base;
};

DQ const DQ_kinematics::base(DQ_kinematics param_dq_kin) {
    return param_dq_kin.base();
};

matrix <double> const DQ_kinematics::C8() {
	matrix <double> diag_C8(8,8);
    diag_C8(0,0) = -1; diag_C8(0,1) = 0; diag_C8(0,2) = 0; diag_C8(0,3) = 0;
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

    diag_C8(4,4) = -1; diag_C8(4,5) = 0; diag_C8(4,6) = 0; diag_C8(4,7) = 0;
    diag_C8(5,4) = 0; diag_C8(5,5) = -1; diag_C8(5,6) = 0; diag_C8(5,7) = 0;
    diag_C8(6,4) = 0; diag_C8(6,5) = 0; diag_C8(6,6) = -1; diag_C8(6,7) = 0;
    diag_C8(7,4) = 0; diag_C8(7,5) = 0; diag_C8(7,6) = 0; diag_C8(7,7) = -1;
	return diag_C8;
};

matrix <double> const DQ_kinematics::C8(DQ_kinematics param_dq_kin) {
    return param_dq_kin.C8();
};

matrix <double> const DQ_kinematics::C4() {
	matrix <double> diag_C4(4,4);
    diag_C4(0,0) = -1; diag_C4(0,1) = 0; diag_C4(0,2) = 0; diag_C4(0,3) = 0;
    diag_C4(1,0) = 0; diag_C4(1,1) = -1; diag_C4(1,2) = 0; diag_C4(1,3) = 0;
    diag_C4(2,0) = 0; diag_C4(2,1) = 0; diag_C4(2,2) = -1; diag_C4(2,3) = 0;
    diag_C4(3,0) = 0; diag_C4(3,1) = 0; diag_C4(3,2) = 0; diag_C4(3,3) = -1;
    return diag_C4;
};

matrix <double> const DQ_kinematics::C4(DQ_kinematics param_dq_kin) {
    return param_dq_kin.C4();
};

DQ const DQ_kinematics::set_base(DQ new_base) {
    curr_base = new_base;
    return curr_base;
};

DQ const DQ_kinematics::set_base(DQ_kinematics param_dq_kin, DQ new_base) {
    return param_dq_kin.set_base(new_base);
};

DQ const DQ_kinematics::fkm(vector <double> theta_vec) {
    DQ_kinematics aux_dq_kin(dq_kin);
    if((int)theta_vec.size() != (aux_dq_kin.links() - aux_dq_kin.n_dummy()) ) {
        //erro
        cout << "\n INCORRECT NUMBER OF JOINT VARIABLES \n";
    }
    DQ q(1);
    int j = 0;
    for (int i = 0; i < aux_dq_kin.links(); i++) {
        if(aux_dq_kin.dummy()(i) == 1) {
            q = q * dh2dq(aux_dq_kin, 0, i);
            j = j + 1;
        }
        else
            q = q * dh2dq(aux_dq_kin, theta_vec(i-j), i);
    }
    q = aux_dq_kin.base() * q;
    return q;
};

DQ const DQ_kinematics::fkm(DQ_kinematics param_dq_kin, vector <double> theta_vec) {
    return param_dq_kin.fkm(theta_vec);
};

DQ const DQ_kinematics::fkm(vector <double> theta_vec, int ith) {
    DQ_kinematics aux_dq_kin(dq_kin);
    if((int)theta_vec.size() != (aux_dq_kin.links() - aux_dq_kin.n_dummy()) ) {
        //erro
        cout << "\n INCORRECT NUMBER OF JOINT VARIABLES \n";
    }
    DQ q(1);
    int j = 0;
    for (int i = 0; i < ith; i++) {
        if(aux_dq_kin.dummy()(i) == 1) {
            q = q * dh2dq(aux_dq_kin, 0, i);
            j = j + 1;
        }
        else
            q = q * dh2dq(aux_dq_kin, theta_vec(i-j), i);
    }
    q = aux_dq_kin.base() * q;
    return q;
};

DQ const DQ_kinematics::fkm(DQ_kinematics param_dq_kin, vector <double> theta_vec, int ith) {
    return param_dq_kin.fkm(theta_vec, ith);
};

DQ const DQ_kinematics::dh2dq(double theta_ang, int link_i) {
    DQ_kinematics aux_dq_kin(dq_kin);
    vector <double> q(8);

    double d = aux_dq_kin.d()(link_i);
    double a = aux_dq_kin.a()(link_i);
    double alpha = aux_dq_kin.alpha()(link_i);
    //std::string standard = "standard";

    if(aux_dq_kin.convention() == "standard") {

        q(1)=cos((theta_ang + aux_dq_kin.theta()(link_i) )/2)*cos(alpha/2);
        q(2)=cos((theta_ang + aux_dq_kin.theta()(link_i) )/2)*sin(alpha/2);
        q(3)=sin((theta_ang + aux_dq_kin.theta()(link_i) )/2)*sin(alpha/2);
        q(4)=sin((theta_ang + aux_dq_kin.theta()(link_i) )/2)*cos(alpha/2);
        double d2=d/2;
        double a2=a/2;
        q(5)= -d2*q(4) - a2*q(2);
        q(6)= -d2*q(3) + a2*q(1);
        q(7)= d2*q(2) + a2*q(4);
        q(8)= d2*q(1) - a2*q(3);
    }
    else{

        double h1 = cos((theta_ang + aux_dq_kin.theta()(link_i) )/2)*cos(alpha/2);
        double h2 = cos((theta_ang + aux_dq_kin.theta()(link_i) )/2)*sin(alpha/2);
        double h3 = sin((theta_ang + aux_dq_kin.theta()(link_i) )/2)*sin(alpha/2);
        double h4 = sin((theta_ang + aux_dq_kin.theta()(link_i) )/2)*cos(alpha/2);
        q(1)= h1;
        q(2)= h2;
        q(3)= -h3;
        q(4)= h4;
        double d2=d/2;
        double a2=a/2;
        q(5)=-d2*h4 - a2*h2;
        q(6)=-d2*h3 + a2*h1;
        q(7)=-(d2*h2 + a2*h4);
        q(8)=d2*h1 - a2*h3;
    }
    return DQ(q);
};

DQ const DQ_kinematics::dh2dq(DQ_kinematics param_dq_kin, double theta_ang, int link_i) {
    return param_dq_kin.dh2dq(theta_ang, link_i);
};

DQ const DQ_kinematics::get_p(vector <double> q) {
    vector <double> p(8);
    p(1) = 0;
    p(2)=q(2)*q(4) + q(1)*q(3);
    p(3)=q(3)*q(4) - q(1)* q(2);
    p(4)=(q(4)*q(4)-q(3)*q(3)-q(2)*q(2)+q(1)*q(1))/2;
    p(5)=0;
    p(6)=q(2)*q(8)+q(6)*q(4)+q(1)*q(7)+q(5)*q(3);
    p(7)=q(3)*q(8)+q(7)*q(4)-q(1)*q(6)-q(5)*q(2);
    p(8)=q(4)*q(8)-q(3)*q(7)-q(2)*q(6)+q(1)*q(5);
    return DQ(p);
};

DQ const DQ_kinematics::get_p(DQ_kinematics param_dq_kin, vector <double> q) {
    return param_dq_kin.get_p(q);
};

matrix <double> const DQ_kinematics::jacobian(vector <double> theta_vec) {
    DQ_kinematics aux_dq_kin(dq_kin);
    DQ q_effector = aux_dq_kin.fkm(theta_vec);

    DQ p;
    DQ q(1);

    matrix <double> J(8,(aux_dq_kin.links() - aux_dq_kin.n_dummy()) );
    for (unsigned int i = 0; i < J.size1(); i++) {
        for(unsigned int j = 0; j < J.size2(); j++) {
            J(i,j) = 0;
        }
    }
    int ith = -1;
    for(int i = -1; i < aux_dq_kin.links()-2; i++) {
        if(aux_dq_kin.dummy()(i+1) == 0) {
            if(aux_dq_kin.convention() == "standard")
                p = aux_dq_kin.get_p(q.q);
            else {
                DQ w(0, 0, -sin(aux_dq_kin.alpha()(i+1)), cos(aux_dq_kin.alpha()(i+1)), 0, 0, -aux_dq_kin.a()(i+1)*cos(aux_dq_kin.alpha()(i+1)), -aux_dq_kin.a()(i+1)*sin(aux_dq_kin.alpha()(i+1)));
                p =0.5 * q * w * q.conj();
                }

            q = q * aux_dq_kin.dh2dq(theta_vec(i+1),(i+1));
            DQ aux_j = p * q_effector;
            for(int i = 0; i < 8; i++) {
            J(i,ith+1) = aux_j.q(i);
            }
            ith = ith+1;
        }
    }
    J = prod(DQ::Hplus8(aux_dq_kin.base()), J);
    return J;
};

matrix <double> const DQ_kinematics::jacobian(DQ_kinematics param_dq_kin, vector <double> theta_vec) {
    return param_dq_kin.jacobian(theta_vec);
};

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

matrix <double> const DQ_kinematics::jacobp(DQ_kinematics param_dq_kin, matrix <double> param_jacobian, vector <double> x) {
    return param_dq_kin.jacobp(param_jacobian, x);
};

matrix <double> const DQ_kinematics::jacobd(matrix <double> param_jacobian, vector <double> x) {
    DQ dq_x(x);
    DQ p = DQ::translation(dq_x);
    matrix <double> Jp = DQ_kinematics::jacobp(param_jacobian, x);
    matrix <double> Jd = 2 * prod(DQ::vec4(p), Jp);
    return Jd;
};

matrix <double> const DQ_kinematics::jacobd(DQ_kinematics param_dq_kin, matrix <double> param_jacobian, vector <double> x) {
    return param_dq_kin.jacobd(param_jacobian, x);
};
