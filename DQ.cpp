#include "DQ.h"
#include <iostream>
#include<math.h>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/lexical_cast.hpp>

using namespace boost::numeric::ublas;
using std::cout;

DQ::DQ() {
    q.resize(8);
    for(int n = 0; n < 8; n++){
        q(n) = 0;
	}
};

/*
* DQ(vector <double> v)
* Creates a DQ object with primary and dual part filled by scalars of a double boost vector.
*/
DQ::DQ(vector <double> v) {
    q.resize(8);
    if(v.size() == 8 || v.size() == 4 || v.size() == 1) {
		for(unsigned int n = 0; n < v.size() ; n++) {
		    if( fabs(v(n)) < DQ::threshold())
		    q(n) = 0;
		    else
            q(n) = v(n);
		}
		for(int n = v.size(); n < 8 ; n++) {
            q(n) = 0;
        }
	}
	else
		//error
		cout << "\n" << "ERROR: VECTOR V SIZE NEEDS TO BE 8, 4 OR 1";
};

DQ::DQ(double scalar) {
    q.resize(8);
    for(int n = 0; n < 8; n++) {
        q(n) = 0;
    }
    if( fabs(scalar) >= DQ::threshold())
    q(0) = scalar;
};

DQ::DQ(double q0,double q1,double q2,double q3,double q4,double q5,double q6,double q7) {
    q.resize(8);
    q(0) = q0;
    q(1) = q1;
    q(2) = q2;
    q(3) = q3;
    q(4) = q4;
    q(5) = q5;
    q(6) = q6;
    q(7) = q7;
    for(int n = 0; n < 8; n++) {
            if(fabs(q(n)) < DQ::threshold() )
                q(n) = 0;
        }
};
/*
* ~DQ()
* Destructor
*/
DQ::~DQ(){};


// Public constant methods

DQ const DQ::E() {
	return set_E();
};

DQ const DQ::i() {
	return set_i();
};

DQ const DQ::j() {
	return set_j();
};

DQ const DQ::k() {
	return set_k();
};

DQ const DQ::P() {
    return set_P();
};

DQ const DQ::D() {
    return set_D();
};

DQ const DQ::Re() {
    return set_Re();
};

DQ const DQ::Im() {
    return set_Im();
};

DQ const DQ::conj() {
    return set_conj();
};

DQ const DQ::norm() {
    return set_norm();
};

DQ const DQ::inv() {
    return set_inv();
};

DQ const DQ::translation() {
    return set_translation();
};

DQ const DQ::rotation_axis() {
    return set_rotation_axis();
};

DQ const DQ::log() {
    return set_log();
};

DQ const DQ::exp() {
    return set_exp();
};

matrix <double> const DQ::Hplus4() {
    return set_Hplus4();
};

matrix <double> const DQ::Hminus4() {
    return set_Hminus4();
};

matrix <double> const DQ::Hplus8() {
    return set_Hplus8();
};

matrix <double> const DQ::Hminus8() {
    return set_Hminus8();
};

matrix <double> const DQ::vec4() {
    return set_vec4();
};

matrix <double> const DQ::vec8() {
    return set_vec8();
};


// Private methods: these are the auxiliar methods used by the public methods.

DQ DQ::set_E() {
    vector <double> v(8);
    for(int n = 0; n < 8; n++){
        v(n) = 0;
    }
    v(4) = 1;
    return DQ(v);
};

DQ DQ::set_i() {
    vector <double> v(8);
    for(int n = 0; n < 8; n++){
        v(n) = 0;
    }
    v(1) = 1;
    return DQ(v);
};

DQ DQ::set_j() {
    vector <double> v(8);
    for(int n = 0; n < 8; n++){
        v(n) = 0;
    }
    v(2) = 1;
    return DQ(v);
};

DQ DQ::set_k() {
    vector <double> v(8);
    for(int n = 0; n < 8; n++){
        v(n) = 0;
    }
    v(3) = 1;
    return DQ(v);
};

DQ DQ::set_P() {
    vector <double> v(8);
    for(int n = 0; n < 4; n++){
        v(n) = q(n);
	}
	for(int n = 4; n < 8; n++){
        v(n) = 0;
	}
    return DQ(v);
};

DQ DQ::set_D() {
    vector <double> v(8);
    for(int n = 0; n < 4; n++){
        v(n) = q(n+4);
	}
		for(int n = 4; n < 8; n++){
        v(n) = 0;
	}
    return DQ(v);
};

DQ DQ::set_Re() {
    vector <double> v(8);
    for(int n = 0; n < 8; n++){
        v(n) = 0;
	}
	v(0) = q(0);
	v(4) = q(4);
    return DQ(v);
};

DQ DQ::set_Im() {
    vector <double> v(8);
    for(int n = 0; n < 8; n++){
        v(n) = q(n);
	}
	v(0) = 0;
	v(4) = 0;
	return DQ(v);
};

DQ DQ::set_conj() {
    vector <double> v(8);
    for(int n = 0; n < 8; n++){
        v(n) = -q(n);
	}
	v(0) = q(0);
	v(4) = q(4);
	return DQ(v);
};

DQ DQ::set_norm() {
    DQ aux;
    DQ norm;

    for(int n = 0; n < 8; n++) {
        aux.q(n) = q(n);
	}
	if(aux.P() == 0)  //Primary == 0
         return norm;
    else {
        aux = aux.conj() * aux;
        aux.q(1) = sqrt(aux.q(1));
        aux.q(5) = aux.q(5)/(2*aux.q(1));

        for(int n = 0; n < 8; n++) {
            if(fabs(aux.q(n)) < DQ::threshold() )
                aux.q(n) = 0;
        }
        norm = aux;
        return norm;
    }
};

DQ DQ::set_inv() {
    DQ aux;
    DQ aux2;

    for(int n = 0; n < 8; n++) {
        aux.q(n) = q(n);
	}
	aux2 = aux * aux.conj(); //(dq norm)^2
	DQ inv((1/aux2.q(0)),0,0,0,(-aux2.q(4)/(aux2.q(0)*aux2.q(0))),0,0,0);

	return (aux.conj() * inv);
};

DQ DQ::set_translation() {
    DQ aux;
    DQ aux2;

    for(int n = 0; n < 8; n++) {
        aux.q(n) = q(n);
	}
	if (aux.norm() != 1) {
        cout << "ERROR IN OPERATION: NOT A UNIT DUAL QUATERNION";
        return aux;
    }
    else {
        aux2 = aux.P();
        return (2 * aux.D() * aux2.conj() );
    }

};

DQ DQ::set_rotation_axis() {
    DQ aux;
    for(int n = 0; n < 8; n++) {
        aux.q(n) = q(n);
	}
	if (aux.norm() != 1) {
        cout << "ERROR IN OPERATION: NOT A UNIT DUAL QUATERNION";
        return aux;
    }
    else {
        double phi = acos(aux.q(0));
        if(phi == 0)
            return aux.k();
        else {
            aux = aux.P();
            return ( aux.Im() * (1/sin(phi)) );
        }
    }
};

DQ DQ::set_log() {
    DQ aux;
    for(int n = 0; n < 8; n++) {
        aux.q(n) = q(n);
	}
	if (aux.norm() != 1) {
        cout << "ERROR IN OPERATION: NOT A UNIT DUAL QUATERNION";
        return aux;
    }
    else {
        DQ p = acos(aux.q(0)) * aux.rotation_axis();
        DQ d = 0.5 * aux.translation();
        DQ lg(p.q(0),p.q(1),p.q(2),p.q(3),d.q(0),d.q(1),d.q(2),d.q(3));
        return lg;
    }
};

DQ DQ::set_exp() {
    DQ dq;
    DQ phi;
    DQ prim;
    for(int n = 0; n < 8; n++) {
        dq.q(n) = q(n);
	}
	if (dq.norm() != 1) {
        cout << "ERROR IN OPERATION: NOT A UNIT DUAL QUATERNION";
        return dq;
    }
    else {
    phi = dq.P();
    phi = dq.norm();
        if(phi != 0)
            prim = cos(phi.q(0)) + (sin(phi.q(0))/phi.q(0))*dq.P();
        else {
            DQ aux(1);
            prim = aux;
        }

        if(prim.q(0) < 0)
            return ( -1*(prim + prim.E()*dq.D()*prim) );
        else
            return ( prim + prim.E()*dq.D()*prim );
    }
};

double DQ::threshold() {
	const double threshold = 0.000000000001;
	return threshold;
};

matrix <double> DQ::set_Hplus4() {
    matrix <double> op_Hplus4(4,4);
    op_Hplus4(0,0) = q(0); op_Hplus4(0,1) = -q(1); op_Hplus4(0,2) = -q(2); op_Hplus4(0,3) = -q(3);
    op_Hplus4(1,0) = q(1); op_Hplus4(1,1) =  q(0); op_Hplus4(1,2) = -q(3); op_Hplus4(1,3) =  q(2);
    op_Hplus4(2,0) = q(2); op_Hplus4(2,1) =  q(3); op_Hplus4(2,2) =  q(0); op_Hplus4(2,3) = -q(1);
    op_Hplus4(3,0) = q(3); op_Hplus4(3,1) = -q(2); op_Hplus4(3,2) =  q(1); op_Hplus4(3,3) =  q(0);
    return op_Hplus4;
};

matrix <double> DQ::set_Hminus4() {
    matrix <double> op_Hminus4(4,4);
    op_Hminus4(0,0) = q(0); op_Hminus4(0,1) = -q(1); op_Hminus4(0,2) = -q(2); op_Hminus4(0,3) = -q(3);
    op_Hminus4(1,0) = q(1); op_Hminus4(1,1) =  q(0); op_Hminus4(1,2) =  q(3); op_Hminus4(1,3) = -q(2);
    op_Hminus4(2,0) = q(2); op_Hminus4(2,1) = -q(3); op_Hminus4(2,2) =  q(0); op_Hminus4(2,3) =  q(1);
    op_Hminus4(3,0) = q(3); op_Hminus4(3,1) =  q(2); op_Hminus4(3,2) = -q(1); op_Hminus4(3,3) =  q(0);
    return op_Hminus4;
};

matrix <double> DQ::set_Hplus8() {
    matrix <double> op_Hplus8(8,8);
    op_Hplus8(0,0) = q(0); op_Hplus8(0,1) = -q(1); op_Hplus8(0,2) = -q(2); op_Hplus8(0,3) = -q(3);
    op_Hplus8(1,0) = q(1); op_Hplus8(1,1) =  q(0); op_Hplus8(1,2) = -q(3); op_Hplus8(1,3) =  q(2);
    op_Hplus8(2,0) = q(2); op_Hplus8(2,1) =  q(3); op_Hplus8(2,2) =  q(0); op_Hplus8(2,3) = -q(1);
    op_Hplus8(3,0) = q(3); op_Hplus8(3,1) = -q(2); op_Hplus8(3,2) =  q(1); op_Hplus8(3,3) =  q(0);

    op_Hplus8(0,4) = 0; op_Hplus8(0,5) = 0; op_Hplus8(0,6) = 0; op_Hplus8(0,7) = 0;
    op_Hplus8(1,4) = 0; op_Hplus8(1,5) = 0; op_Hplus8(1,6) = 0; op_Hplus8(1,7) = 0;
    op_Hplus8(2,4) = 0; op_Hplus8(2,5) = 0; op_Hplus8(2,6) = 0; op_Hplus8(2,7) = 0;
    op_Hplus8(3,4) = 0; op_Hplus8(3,5) = 0; op_Hplus8(3,6) = 0; op_Hplus8(3,7) = 0;

    op_Hplus8(4,0) = q(4); op_Hplus8(4,1) = -q(5); op_Hplus8(4,2) = -q(6); op_Hplus8(4,3) = -q(7);
    op_Hplus8(5,0) = q(5); op_Hplus8(5,1) =  q(4); op_Hplus8(5,2) = -q(7); op_Hplus8(5,3) =  q(6);
    op_Hplus8(6,0) = q(6); op_Hplus8(6,1) =  q(7); op_Hplus8(6,2) =  q(4); op_Hplus8(6,3) = -q(5);
    op_Hplus8(7,0) = q(7); op_Hplus8(7,1) = -q(6); op_Hplus8(7,2) =  q(5); op_Hplus8(7,3) =  q(4);

    op_Hplus8(4,4) = q(0); op_Hplus8(4,5) = -q(1); op_Hplus8(4,6) = -q(2); op_Hplus8(4,7) = -q(3);
    op_Hplus8(5,4) = q(1); op_Hplus8(5,5) =  q(0); op_Hplus8(5,6) = -q(3); op_Hplus8(5,7) =  q(2);
    op_Hplus8(6,4) = q(2); op_Hplus8(6,5) =  q(3); op_Hplus8(6,6) =  q(0); op_Hplus8(6,7) = -q(1);
    op_Hplus8(7,4) = q(3); op_Hplus8(7,5) = -q(2); op_Hplus8(7,6) =  q(1); op_Hplus8(7,7) =  q(0);
    return op_Hplus8;
};

matrix <double> DQ::set_Hminus8() {
    matrix <double> op_Hminus8(8,8);
    op_Hminus8(0,0) = q(0); op_Hminus8(0,1) = -q(1); op_Hminus8(0,2) = -q(2); op_Hminus8(0,3) = -q(3);
    op_Hminus8(1,0) = q(1); op_Hminus8(1,1) =  q(0); op_Hminus8(1,2) =  q(3); op_Hminus8(1,3) = -q(2);
    op_Hminus8(2,0) = q(2); op_Hminus8(2,1) = -q(3); op_Hminus8(2,2) =  q(0); op_Hminus8(2,3) =  q(1);
    op_Hminus8(3,0) = q(3); op_Hminus8(3,1) =  q(2); op_Hminus8(3,2) = -q(1); op_Hminus8(3,3) =  q(0);

    op_Hminus8(0,4) = 0; op_Hminus8(0,5) = 0; op_Hminus8(0,6) = 0; op_Hminus8(0,7) = 0;
    op_Hminus8(1,4) = 0; op_Hminus8(1,5) = 0; op_Hminus8(1,6) = 0; op_Hminus8(1,7) = 0;
    op_Hminus8(2,4) = 0; op_Hminus8(2,5) = 0; op_Hminus8(2,6) = 0; op_Hminus8(2,7) = 0;
    op_Hminus8(3,4) = 0; op_Hminus8(3,5) = 0; op_Hminus8(3,6) = 0; op_Hminus8(3,7) = 0;

    op_Hminus8(4,0) = q(4); op_Hminus8(4,1) = -q(5); op_Hminus8(4,2) = -q(6); op_Hminus8(4,3) = -q(7);
    op_Hminus8(5,0) = q(5); op_Hminus8(5,1) =  q(4); op_Hminus8(5,2) =  q(7); op_Hminus8(5,3) = -q(6);
    op_Hminus8(6,0) = q(6); op_Hminus8(6,1) = -q(7); op_Hminus8(6,2) =  q(4); op_Hminus8(6,3) =  q(5);
    op_Hminus8(7,0) = q(7); op_Hminus8(7,1) =  q(6); op_Hminus8(7,2) = -q(5); op_Hminus8(7,3) =  q(4);

    op_Hminus8(4,4) = q(0); op_Hminus8(4,5) = -q(1); op_Hminus8(4,6) = -q(2); op_Hminus8(4,7) = -q(3);
    op_Hminus8(5,4) = q(1); op_Hminus8(5,5) =  q(0); op_Hminus8(5,6) =  q(3); op_Hminus8(5,7) = -q(2);
    op_Hminus8(6,4) = q(2); op_Hminus8(6,5) = -q(3); op_Hminus8(6,6) =  q(0); op_Hminus8(6,7) =  q(1);
    op_Hminus8(7,4) = q(3); op_Hminus8(7,5) =  q(2); op_Hminus8(7,6) = -q(1); op_Hminus8(7,7) =  q(0);
    return op_Hminus8;
};

matrix <double> DQ::set_vec4() {
    matrix <double> op_vec4(4,1);
    op_vec4(0,0) = q(0);
    op_vec4(1,0) = q(1);
    op_vec4(2,0) = q(2);
    op_vec4(3,0) = q(3);
    return op_vec4;
};

matrix <double> DQ::set_vec8() {
    matrix <double> op_vec8(8,1);
    op_vec8(0,0) = q(0);
    op_vec8(1,0) = q(1);
    op_vec8(2,0) = q(2);
    op_vec8(3,0) = q(3);
    op_vec8(4,0) = q(4);
    op_vec8(5,0) = q(5);
    op_vec8(6,0) = q(6);
    op_vec8(7,0) = q(7);
    return op_vec8;
};

void DQ::display() {
    vector <double> v(8);
    for(int n = 0; n < 8; n++){
        v(n) = q(n);
	}
	DQ dq(v);

	has_primary_element = 0;
	has_dual_element = 0;
    //disp([inputname(1),' = ']) EXIBE O NOME DO DQ, ACHAR UMA FUNC CORRESP EM C++
    std::string s, sd;

    s = build_string(dq,0);
    sd = build_string(dq,4);

    if (has_primary_element == 1) {
        s = "(" + s + ")";
        if (has_dual_element == 1) {
            sd = " + E*(" + sd + ")";
            has_dual_element = 0;
        }
    }

    if (has_dual_element == 1) {
        sd = "E*(" + sd + ")";
        if (has_primary_element == 1) {
            s = "(" + s + ")";
            sd = " + " + sd;
        }
    }

    if ((has_primary_element + has_dual_element) == 0)
        s = "0";
    cout << " = " << s << sd << "\n";
};

std::string DQ::build_string(DQ dq, int shift) {
    std::string s, aux;
    s = "";
    aux = " ijk";

    int has_element = 0;

    for(int n = 0; n < 4; n++) {
        if (fabs(dq.q(n+shift) ) > dq.threshold() ) { //To avoid printing values very close to zero
            if (dq.q(n+shift) < 0)
                s = s + " - ";
            else if (has_element != 0)
                s = s + " + ";

            if (n == 0)
                s = s + boost::lexical_cast<std::string>(fabs(dq.q(n+shift) ) );
            else
                s = s + boost::lexical_cast<std::string>(fabs(dq.q(n+shift) ) ) + aux[n];

            if (shift == 0) {
            has_primary_element = 1;
            has_element = 1;
            }
            else {
            has_dual_element = 1;
            has_element = 1;
            }
        }
    }
    return s;
};

//Overloaded operators definitions

//Operator (+) Overload
DQ operator+(DQ dq1, DQ dq2) {
    DQ dq;
    for(int n = 0; n<8; n++) {
        dq.q(n) = dq1.q(n) + dq2.q(n);
    }
    for(int n = 0; n < 8; n++) {
            if(fabs(dq.q(n)) < DQ::threshold() )
                dq.q(n) = 0;
        }
    return dq;
};

//Overload (+) for int type scalar
DQ operator+(DQ dq, int scalar) {
    DQ dq_scalar(scalar);
    return (dq + dq_scalar);
};
DQ operator+(int scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar + dq);
};

//Overload (+) for float type scalar
DQ operator+(DQ dq, float scalar) {
    DQ dq_scalar(scalar);
    return (dq + dq_scalar);
};
DQ operator+(float scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar + dq);
};

//Overload (+) for double type scalar
DQ operator+(DQ dq, double scalar) {
    DQ dq_scalar(scalar);
    return (dq + dq_scalar);
};
DQ operator+(double scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar + dq);
};

// Operator (-) overload
DQ operator-(DQ dq1, DQ dq2){
    DQ dq;
    for(int n = 0; n<8; n++) {
        dq.q(n) = dq1.q(n) - dq2.q(n);
    }
    for(int n = 0; n < 8; n++) {
            if(fabs(dq.q(n)) < DQ::threshold() )
                dq.q(n) = 0;
        }
    return dq;
};

// Overload (-) for int type scalar
DQ operator-(DQ dq, int scalar){
    DQ dq_scalar(scalar);
    return (dq - dq_scalar);
};
DQ operator-(int scalar, DQ dq){
    DQ dq_scalar(scalar);
    return (dq_scalar - dq);
};

//Overload (-) for float type scalar
DQ operator-(DQ dq, float scalar){
    DQ dq_scalar(scalar);
    return (dq - dq_scalar);
};
DQ operator-(float scalar, DQ dq){
    DQ dq_scalar(scalar);
    return (dq_scalar - dq);
};

//Overload (-) for double type scalar
DQ operator-(DQ dq, double scalar){
    DQ dq_scalar(scalar);
    return (dq - dq_scalar);
};
DQ operator-(double scalar, DQ dq){
    DQ dq_scalar(scalar);
    return (dq_scalar - dq);
};

// Operator (*) overload
DQ operator*(DQ dq1, DQ dq2){
    DQ dq;

    dq.q(0) = dq1.q(0)*dq2.q(0) - dq1.q(1)*dq2.q(1) - dq1.q(2)*dq2.q(2) - dq1.q(3)*dq2.q(3);
    dq.q(1) = dq1.q(0)*dq2.q(1) + dq1.q(1)*dq2.q(0) + dq1.q(2)*dq2.q(3) - dq1.q(3)*dq2.q(2);
    dq.q(2) = dq1.q(0)*dq2.q(2) - dq1.q(1)*dq2.q(3) + dq1.q(2)*dq2.q(0) + dq1.q(3)*dq2.q(1);
    dq.q(3) = dq1.q(0)*dq2.q(3) + dq1.q(1)*dq2.q(2) - dq1.q(2)*dq2.q(1) + dq1.q(3)*dq2.q(0);

    dq.q(4) = dq1.q(0)*dq2.D().q(0) - dq1.q(1)*dq2.D().q(1) - dq1.q(2)*dq2.D().q(2) - dq1.q(3)*dq2.D().q(3);
    dq.q(5) = dq1.q(0)*dq2.D().q(1) + dq1.q(1)*dq2.D().q(0) + dq1.q(2)*dq2.D().q(3) - dq1.q(3)*dq2.D().q(2);
    dq.q(6) = dq1.q(0)*dq2.D().q(2) - dq1.q(1)*dq2.D().q(3) + dq1.q(2)*dq2.D().q(0) + dq1.q(3)*dq2.D().q(1);
    dq.q(7) = dq1.q(0)*dq2.D().q(3) + dq1.q(1)*dq2.D().q(2) - dq1.q(2)*dq2.D().q(1) + dq1.q(3)*dq2.D().q(0);

    dq.q(4) = dq.q(4) + dq1.D().q(0)*dq2.P().q(0) - dq1.D().q(1)*dq2.P().q(1) - dq1.D().q(2)*dq2.P().q(2) - dq1.D().q(3)*dq2.P().q(3);
    dq.q(5) = dq.q(5) + dq1.D().q(0)*dq2.P().q(1) + dq1.D().q(1)*dq2.P().q(0) + dq1.D().q(2)*dq2.P().q(3) - dq1.D().q(3)*dq2.P().q(2);
    dq.q(6) = dq.q(6) + dq1.D().q(0)*dq2.P().q(2) - dq1.D().q(1)*dq2.P().q(3) + dq1.D().q(2)*dq2.P().q(0) + dq1.D().q(3)*dq2.P().q(1);
    dq.q(7) = dq.q(7) + dq1.D().q(0)*dq2.P().q(3) + dq1.D().q(1)*dq2.P().q(2) - dq1.D().q(2)*dq2.P().q(1) + dq1.D().q(3)*dq2.P().q(0);

    for(int n = 0; n < 8; n++) {
            if(fabs(dq.q(n)) < DQ::threshold() )
                dq.q(n) = 0;
        }

    return dq;
};

// Overload (*) for int type scalar
DQ operator*(DQ dq, int scalar){
    DQ dq_scalar(scalar);
    return (dq * dq_scalar);
};
DQ operator*(int scalar, DQ dq){
    DQ dq_scalar(scalar);
    return (dq_scalar * dq);
};

// Overload (*) for float type scalar
DQ operator*(DQ dq, float scalar){
    DQ dq_scalar(scalar);
    return (dq * dq_scalar);
};
DQ operator*(float scalar, DQ dq){
    DQ dq_scalar(scalar);
    return (dq_scalar * dq);
};

// Overload (*) for double type scalar
DQ operator*(DQ dq, double scalar){
    DQ dq_scalar(scalar);
    return (dq * dq_scalar);
};
DQ operator*(double scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar * dq);
};

// Operator (==) overload
bool DQ::operator==(DQ dq2) {
    for(int n = 0; n<8; n++) {
        if(fabs(q(n) - dq2.q(n)) > DQ::threshold() )
        return false; //elements of Dual Quaternion different of scalar
    }
    return true; //elements of Dual Quaternion equal to scalar
};

// Overload (==) for int type scalar
bool operator==(DQ dq, int scalar) {
    DQ dq_scalar(scalar);
    return (dq == dq_scalar);
};

bool operator==(int scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar == dq);
};

// Overload (==) for float type scalar
bool operator==(DQ dq, float scalar) {
    DQ dq_scalar(scalar);
    return (dq == dq_scalar);
};

bool operator==(float scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar == dq);
};

// Overload (==) for double type scalar
bool operator==(DQ dq, double scalar) {
    DQ dq_scalar(scalar);
    return (dq == dq_scalar);
};

bool operator==(double scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar == dq);
};

// Operator (!=) overload
bool DQ::operator!=(DQ dq2) {
    for(int n = 0; n<8; n++){
        if(fabs(q(n) - dq2.q(n)) > DQ::threshold() )
        return true; //elements of Dual Quaternion different of scalar
    }
    return false; //elements of Dual Quaternion equal to scalar
};

// Overload (!=) for int type scalar
bool operator!=(DQ dq, int scalar) {
    DQ dq_scalar(scalar);
    return (dq == dq_scalar);
};

bool operator!=(int scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar != dq);
};

// Overload (!=) for float type scalar
bool operator!=(DQ dq, float scalar) {
    DQ dq_scalar(scalar);
    return (dq == dq_scalar);
};

bool operator!=(float scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar != dq);
};

// Overload (!=) for double type scalar
bool operator!=(DQ dq, double scalar) {
    DQ dq_scalar(scalar);
    return (dq == dq_scalar);
};

bool operator!=(double scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar != dq);
};

DQ operator^(DQ dq1, double m) {
    if (dq1.norm() != 1) {
        cout << "ERROR IN OPERATION: NOT A UNIT DUAL QUATERNION";
        return dq1;
    }
    else {
        DQ dq;
        dq = m * dq1.log();
        dq = dq.exp();
        for(int n = 0; n < 8; n++) {
                if(fabs(dq.q(n)) < DQ::threshold() )
                    dq.q(n) = 0;
            }
        return dq;
    }
};
