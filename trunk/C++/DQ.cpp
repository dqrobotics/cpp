#include "DQ.h"
#include <iostream>
#include <iomanip>
#include<math.h>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace boost::numeric::ublas;
using std::cout;

/**
* DQ Default constructor, dont need parameters.
*
* Returns a DQ object with null primary and dual part. All elements of 'q' vector are 0.
* To create a DQ object using this, type: 'DQ dq_object();' or even 'DQ dq_object;'
*/
DQ::DQ() {
    q.resize(8);
    for(int n = 0; n < 8; n++){
        q(n) = 0;
	}
};

/**
* DQ constructor using boost vector
*
* Returns a DQ object with the values of elements equal to the values of elements from a vector 'v' passed to constructor.
* To create a DQ object using this, type: 'DQ dq_object(v);' where 'v' is the double boost vector.
* if 'v' size is bigger than 8, the constructor catches only the 8 eight firs elements of 'v' and distributes on 'vector q'.
* If 'v' size is smaller than 8, the constructor catches the elements of 'v', distributes on 'vector q' and complete the
* rest with zeros. Remember that the first 4 elements are of primary part and the last 4 are of dual part of the quaternion.
* \param vector <double> v contain the values to copied to the attribute q.
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
		cout << "\n" << "WARNING: VECTOR V SIZE IS RECOMMENDED TO BE 8, 4 OR 1";
};

/**
* DQ constructor using 8 scalar elements
*
* Returns a DQ object with the values of vector q equal to the values of the 8 parameters 'q0' to 'q8' passed to constructor.
* To create a DQ object using this, type: 'DQ dq_object(q0,q1,q2,q3,q4,q5,q6,q7);' where 'qn' is a double type scalar.
* \param double q0,q1,q2,q3,q4,q5,q6 and q7 are the values to be copied to the member 'q'.
*/
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

/**
* DQ constructor using 4 scalar elements
*
* Returns a DQ object with the first four values of vector q equal to the values of the 4 parameters 'q0' to 'q4' passed to constructor.
* This represents a quaternion in a dual quaternion form since the last four elements are set to 0.
* To create a DQ object using this, type: 'DQ dq_object(q0,q1,q2,q3);' where 'qn' is a double type scalar.
* \param double q0,q1,q2 and q3 are values to be copied to the member 'q' four first positions.
*/
DQ::DQ(double q0,double q1,double q2,double q3) {
    q.resize(8);
    q(0) = q0;
    q(1) = q1;
    q(2) = q2;
    q(3) = q3;
    q(4) = 0;
    q(5) = 0;
    q(6) = 0;
    q(7) = 0;
    for(int n = 0; n < 4; n++) {
            if(fabs(q(n)) < DQ::threshold() )
                q(n) = 0;
        }
};

/**
* DQ constructor using a scalar element
*
* Returns a DQ object with the first values of vector q equal to the value of the parameter 'scalar' passed to constructor.
* This represents a scalar in a dual quaternion form since the last seven elements ara set to 0.
* To create a DQ object using this, type: 'DQ dq_object(scalar);'.
* \param double scalar is the value to be copied to the member 'q' first position.
*/
DQ::DQ(double scalar) {
    q.resize(8);
    for(int n = 0; n < 8; n++) {
        q(n) = 0;
    }
    if( fabs(scalar) >= DQ::threshold())
    q(0) = scalar;
};

/**
* DQ Destructor
*
* Deletes from memory the DQ object caller. To use this destructor, type: 'dq_object.~DQ();'. Dont need parameters.
*/
DQ::~DQ(){};


// Public constant methods

/**
* Returns a constant DQ object representing the dual unit epsilon.
*
* Creates a dual quaternion with values (0,0,0,0,1,0,0,0) and return. To use this member function, type: 'dq_object.E();'.
* \return A constant DQ object.
* \sa DQ(double q0,double q1,double q2,double q3,double q4,double q5,double q6,double q7).
*/
DQ const DQ::E() {
	return DQ(0,0,0,0,1,0,0,0);
};
/**
* Returns a constant DQ object representing the dual unit epsilon.
* Actually this function does the same as E() changing only the way of calling, which is DQ::E(dq_object).
*/
DQ const DQ::E(DQ dq) {
 return dq.E();
};

/**
* Returns a constant DQ object representing the imaginary unit 'i'.
*
* Creates a dual quaternion with values (0,1,0,0,0,0,0,0) and return. To use this member function, type: 'dq_object.i();'.
* \return A constant DQ object.
* \sa DQ(double q0,double q1,double q2,double q3).
*/
DQ const DQ::i() {
	return DQ(0,1,0,0);
};
/**
* Returns a constant DQ object representing the imaginary unit 'i'.
* Actually this function does the same as i() changing only the way of calling, which is DQ::i(dq_object).
*/
DQ const DQ::i(DQ dq) {
 return dq.i();
};

/**
* Returns a constant DQ object representing the imaginary unit 'j'.
*
* Creates a dual quaternion with values (0,0,1,0,0,0,0,0) and return. To use this member function, type: 'dq_object.j();'.
* \return A constant DQ object.
* \sa DQ(double q0,double q1,double q2,double q3).
*/
DQ const DQ::j() {
	return DQ(0,0,1,0);
};
/**
* Returns a constant DQ object representing the imaginary unit 'j'.
* Actually this function does the same as j() changing only the way of calling, which is DQ::j(dq_object).
*/
DQ const DQ::j(DQ dq) {
 return dq.j();
};

/**
* Returns a constant DQ object representing the imaginary unit 'k'.
*
* Creates a dual quaternion with values (0,0,0,1,0,0,0,0) and return. To use this member function, type: 'dq_object.k();'.
* \return A constant DQ object.
* \sa DQ(double q0,double q1,double q2,double q3).
*/
DQ const DQ::k() {
	return DQ(0,0,0,1);
};
/**
* Returns a constant DQ object representing the imaginary unit 'k'.
* Actually this function does the same as k() changing only the way of calling, which is DQ::k(dq_object).
*/
DQ const DQ::k(DQ dq) {
 return dq.k();
};

/**
* Returns a constant DQ object representing the primary part of the DQ object caller.
*
* Creates a dual quaternion with values (q(0),q(1),q(2),q(3),0,0,0,0) and return. The q elements are from the DQ object caller.
* To use this member function, type: 'dq_object.P();'.
* \return A constant DQ object.
* \sa DQ(double q0,double q1,double q2,double q3).
*/
DQ const DQ::P() {
    return DQ(q(0),q(1),q(2),q(3));
};
/**
* Returns a constant DQ object representing the primary part of the DQ object caller.
* Actually this function does the same as P() changing only the way of calling, which is DQ::P(dq_object).
*/
DQ const DQ::P(DQ dq) {
 return dq.P();
};

/**
* Returns a constant DQ object representing the dual part of the DQ object caller.
*
* Creates a dual quaternion with values (q(4),q(5),q(6),q(7),0,0,0,0) and return. The q elements are from the DQ object caller.
* To use this member function, type: 'dq_object.D();'.
* \return A constant DQ object.
* \sa DQ(double q0,double q1,double q2,double q3).
*/
DQ const DQ::D() {
    return DQ(q(4),q(5),q(6),q(7));
};
/**
* Returns a constant DQ object representing the dual part of the DQ object caller.
* Actually this function does the same as D() changing only the way of calling, which is DQ::D(dq_object).
*/
DQ const DQ::D(DQ dq) {
 return dq.D();
};

/**
* Returns a constant DQ object representing the real part of the DQ object caller.
*
* Creates a dual quaternion with values (q(0),0,0,0,q(4),0,0,0) and return. The q elements are from the DQ object caller.
* To use this member function, type: 'dq_object.Re();'.
* \return A constant DQ object.
* \sa DQ(double q0,double q1,double q2,double q3,double q4,double q5,double q6,double q7).
*/
DQ const DQ::Re() {
    return DQ(q(0),0,0,0,q(4),0,0,0);
};
/**
* Returns a constant DQ object representing the real part of the DQ object caller.
* Actually this function does the same as Re() changing only the way of calling, which is DQ::Re(dq_object).
*/
DQ const DQ::Re(DQ dq) {
 return dq.Re();
};

/**
* Returns a constant DQ object representing the imaginary part of the DQ object caller.
*
* Creates a dual quaternion with values (0,q(1),q(2),q(3),0,q(5),q(6),q(7)) and return. The q elements are from the DQ object caller.
* To use this member function, type: 'dq_object.Im();'.
* \return A constant DQ object.
* \sa DQ(double q0,double q1,double q2,double q3,double q4,double q5,double q6,double q7).
*/
DQ const DQ::Im() {
    return DQ(0,q(1),q(2),q(3),0,q(5),q(6),q(7));
};
/**
* Returns a constant DQ object representing the imaginary part of the DQ object caller.
* Actually this function does the same as Im() changing only the way of calling, which is DQ::Im(dq_object).
*/
DQ const DQ::Im(DQ dq) {
 return dq.Im();
};

/**
* Returns a constant DQ object representing the conjugate of the DQ object caller.
*
* Creates a dual quaternion with values (q(0),-q(1),-q(2),-q(3),q(4),-q(5),-q(6),-q(7)) and return.
* The q elements are from the DQ object caller. To use this member function, type: 'dq_object.conj();'.
* \return A constant DQ object.
* \sa DQ(double q0,double q1,double q2,double q3,double q4,double q5,double q6,double q7).
*/
DQ const DQ::conj() {
    return DQ(q(0),-q(1),-q(2),-q(3),q(4),-q(5),-q(6),-q(7));
};
/**
* Returns a constant DQ object representing the conjugate of the DQ object caller.
* Actually this function does the same as conj() changing only the way of calling, which is DQ::conj(dq_object).
*/
DQ const DQ::conj(DQ dq) {
 return dq.conj();
};

/**
* Returns a constant DQ object representing the norm of the DQ object caller.
*
* Creates a dual quaternion with calculated values for the norm and return a DQ object.
* To use this member function, type: 'dq_object.norm();'.
* \return A constant DQ object.
* \sa DQ().
*/
DQ const DQ::norm() {
    DQ aux;
    DQ norm;

    for(int n = 0; n < 8; n++) {
        aux.q(n) = q(n);
	}
	if(aux.P() == 0)  //Primary == 0
         return norm; // norm = 0
    else {
        // norm calculation
        norm = aux.conj() * aux;
        norm.q(0) = sqrt(norm.q(0));
        norm.q(4) = norm.q(4)/(2*norm.q(4));

        // using threshold to verify zero values in DQ to be returned
        for(int n = 0; n < 8; n++) {
            if(fabs(norm.q(n)) < DQ::threshold() )
                norm.q(n) = 0;
        }

        return norm;
    }
};
/**
* Returns a constant DQ object representing the norm of the DQ object caller.
* Actually this function does the same as norm() changing only the way of calling, which is DQ::norm(dq_object).
*/
DQ const DQ::norm(DQ dq) {
 return dq.norm();
};

/**
* Returns a constant DQ object representing the inverse of the DQ object caller.
*
* Creates a dual quaternion with calculated values for the inverse and return a DQ object.
* To use this member function, type: 'dq_object.inv();'.
* \return A constant DQ object.
* \sa DQ(), DQ(double q0,double q1,double q2,double q3,double q4,double q5,double q6,double q7).
*/
DQ const DQ::inv() {
    DQ aux;
    DQ aux2;

    for(int n = 0; n < 8; n++) {
        aux.q(n) = q(n);
	}
	//inverse calculation
	aux2 = aux * aux.conj(); //(dq norm)^2
	DQ inv((1/aux2.q(0)),0,0,0,(-aux2.q(4)/(aux2.q(0)*aux2.q(0))),0,0,0);
    inv = (aux.conj() * inv);

    // using threshold to verify zero values in DQ to be returned
    for(int n = 0; n < 8; n++) {
        if(fabs(inv.q(n)) < DQ::threshold() )
            inv.q(n) = 0;
    }

	return inv;
};
/**
* Returns a constant DQ object representing the inverse of the DQ object caller.
* Actually this function does the same as inv() changing only the way of calling, which is DQ::inv(dq_object).
*/
DQ const DQ::inv(DQ dq) {
 return dq.inv();
};

/**
* Returns a constant DQ object representing the translation part of the unit DQ object caller.
*
* Creates a dual quaternion with calculated values for the translation part and return a DQ object.
* Assuming  dq=r + DQ.E * p * r * (0.5), the return is p.
* To use this member function, type: 'dq_object.translation();'.
* \return A constant DQ object.
* \sa DQ().
*/
DQ const DQ::translation() {
    DQ aux;
    DQ translation;

    for(int n = 0; n < 8; n++) {
        aux.q(n) = q(n);
	}
    //TODO: Generate a message error here if condition is not satisfied
	// Verify if the object caller is a unit DQ
	try {
        if (aux.norm() != 1) {
            throw 1;
        }

        //translation part calculation
        translation = aux.P();
        translation = (2 * aux.D() * translation.conj() );

        // using threshold to verify zero values in DQ to be returned
        for(int n = 0; n < 8; n++) {
            if(fabs(translation.q(n)) < DQ::threshold() )
                translation.q(n) = 0;
        }

        return translation;
	}
	catch (int i) {
        std::cerr << "ERROR IN TRANSLATION OPERATION: NOT A UNIT DUAL QUATERNION \n";
        system("PAUSE");
        return EXIT_FAILURE;
	}
};
/**
* Returns a constant DQ object representing the translation part of the unit DQ object caller.
* Actually this function does the same as translation() changing only the way of calling, which is DQ::translation(dq_object).
*/
DQ const DQ::translation(DQ dq) {
 return dq.translation();
};

/**
* Returns a constant DQ object representing the rotation axis (nx*i + ny*j + nz*k) of the unit DQ object caller.
*
* Creates a dual quaternion with calculated values for the rotation axis and return a DQ object.
* To use this member function, type: 'dq_object.rot_axis();'.
* \return A constant DQ object.
* \sa DQ().
*/
DQ const DQ::rot_axis() {
    DQ dq;
    DQ rot_axis;
    for(int n = 0; n < 8; n++) {
        dq.q(n) = q(n);
	}
    //TODO: Generate a message error here if condition is not satisfied
	// Verify if the object caller is a unit DQ
	if (dq.norm() != 1) {
        cout << "ERROR IN ROT_AXIS OPERATION: NOT A UNIT DUAL QUATERNION \n";
        return dq;
    }

    else {
        double phi = acos(dq.q(0));
        if(phi == 0)
            return rot_axis.k(); // DQ(0,0,0,1). This is only a convention;
        else {
            //rotation axis calculation
            rot_axis = dq.P();
            rot_axis = ( rot_axis.Im() * (1/sin(phi)) );

            // using threshold to verify zero values in DQ to be returned
            for(int n = 0; n < 8; n++) {
                if(fabs(rot_axis.q(n)) < DQ::threshold() )
                    rot_axis.q(n) = 0;
            }

            return rot_axis;
        }
    }
};
/**
* Returns a constant DQ object representing the rotation axis of the unit DQ object caller.
* Actually this function does the same as rot_axis() changing only the way of calling, which is DQ::rot_axis(dq_object).
*/
DQ const DQ::rot_axis(DQ dq) {
 return dq.rot_axis();
};

/**
* Returns a constant double value representing the rotation angle in rad/s of the unit DQ object caller.
*
* Creates a double value with calculated rotation angle of a unit DQ object and return this angle in rad/s unit.
* To use this member function, type: 'dq_object.rot_angle();'.
* \return A constant double value.
* \sa DQ().
*/
double const DQ::rot_angle() {
    DQ dq;
    double rot_angle;
    for(int n = 0; n < 8; n++) {
        dq.q(n) = q(n);
	}
    //TODO: Generate a message error here if condition is not satisfied
	// Verify if the object caller is a unit DQ
	if (dq.norm() != 1) {
        cout << "ERROR IN ROT_ANGLE OPERATION: NOT A UNIT DUAL QUATERNION \n";
        return 0;
    }

    else {
        //Rotation angle calculation
        rot_angle = 2*acos(dq.q(0));

        return rot_angle;
    }
};
/**
* Returns a constant double value representing the rotation angle in rad/s of the unit DQ object caller.
* Actually this function does the same as rot_angle() changing only the way of calling, which is DQ::rot_angle(dq_object).
*/
DQ const DQ::rot_angle(DQ dq) {
 return dq.rot_angle();
};

/**
* Returns a constant DQ object representing the logaritm of the unit DQ object caller.
*
* Creates a dual quaternion with calculated values for the logaritm and return a DQ object.
* To use this member function, type: 'dq_object.log();'.
* \return A constant DQ object.
* \sa DQ(), DQ(double q0,double q1,double q2,double q3,double q4,double q5,double q6,double q7).
*/
DQ const DQ::log() {
    DQ aux;
    for(int n = 0; n < 8; n++) {
        aux.q(n) = q(n);
	}
    //TODO: Generate a message error here if condition is not satisfied
	// Verify if the object caller is a unit DQ
	if (aux.norm() != 1) {
        cout << "ERROR IN LOG OPERATION: NOT A UNIT DUAL QUATERNION \n";
        return aux;
    }

    else {
        // log calculation
        DQ p = acos(aux.q(0)) * aux.rot_axis(); //primary
        DQ d = 0.5 * aux.translation(); //dual
        DQ log(p.q(0),p.q(1),p.q(2),p.q(3),d.q(0),d.q(1),d.q(2),d.q(3));

        // using threshold to verify zero values in DQ to be returned
        for(int n = 0; n < 8; n++) {
            if(fabs(log.q(n)) < DQ::threshold() )
                log.q(n) = 0;
        }

        return log;
    }
};
/**
* Returns a constant DQ object representing the log of the unit DQ object caller.
* Actually this function does the same as log() changing only the way of calling, which is DQ::log(dq_object).
*/
DQ const DQ::log(DQ dq) {
 return dq.log();
};

/**
* Returns a constant DQ object representing the exponential of the unit DQ object caller.
*
* Creates a dual quaternion with calculated values for the exponential and return a DQ object.
* To use this member function, type: 'dq_object.exp();'.
* \return A constant DQ object.
* \sa DQ().
*/
DQ const DQ::exp() {
        DQ dq;
    DQ phi;
    DQ prim;
    DQ exp;
    for(int n = 0; n < 8; n++) {
        dq.q(n) = q(n);
	}
    //TODO: Generate a message error here if condition is not satisfied
	// Verify if the object caller is a unit DQ
//	if (dq.norm() != 1) {
//        cout << "ERROR IN EXP OPERATION: NOT A UNIT DUAL QUATERNION \n";
//        return dq;
//    }

//    else {
    // exponential calculation
    phi = dq.P();
    phi = dq.norm();
        if(phi != 0)
            prim = cos(phi.q(0)) + (sin(phi.q(0))/phi.q(0))*dq.P();
        else {
            DQ aux(1);
            prim = aux;
        }

        if(prim.q(0) < 0)
            exp = ( -1*(prim + prim.E()*dq.D()*prim) );
        else
            exp = ( prim + prim.E()*dq.D()*prim );

        // using threshold to verify zero values in DQ to be returned
        for(int n = 0; n < 8; n++) {
            if(fabs(exp.q(n)) < DQ::threshold() )
                exp.q(n) = 0;
        }

        return exp;
//    }
};
/**
* Returns a constant DQ object representing the exponential of the unit DQ object caller.
* Actually this function does the same as exp() changing only the way of calling, which is DQ::exp(dq_object).
*/
DQ const DQ::exp(DQ dq) {
 return dq.exp();
};

/**
* Returns a constant DQ object representing the tplus operator applied to the unit DQ object caller.
*
* Creates a dual quaternion with calculated values for the tplus operation and return a DQ object.
* To use this member function, type: 'dq_object.tplus();'.
* \return A constant DQ object.
* \sa DQ().
*/
DQ const DQ::tplus() {
        DQ dq;
    DQ tplus;
    for(int n = 0; n < 8; n++) {
        dq.q(n) = q(n);
	}
    //TODO: Generate a message error here if condition is not satisfied
	// Verify if the object caller is a unit DQ
	if (dq.norm() != 1) {
        cout << "ERROR IN TPLUS OPERATION: NOT A UNIT DUAL QUATERNION \n";
        return dq;
    }

    else {
    // tplus operator calculation
    tplus = dq.P();
    tplus = dq * tplus.conj();

    // using threshold to verify zero values in DQ to be returned
    for(int n = 0; n < 8; n++) {
        if(fabs(tplus.q(n)) < DQ::threshold() )
            tplus.q(n) = 0;
    }

    return tplus;
    }
};
/**
* Returns a constant DQ object representing the tplus operator applied to the unit DQ object caller.
* Actually this function does the same as tplus() changing only the way of calling, which is DQ::tplus(dq_object).
*/
DQ const DQ::tplus(DQ dq) {
 return dq.tplus();
};

/**
* Returns a constant DQ object representing the inverse of the unit DQ object caller under decompositional multiplication.
*
* Creates a dual quaternion with calculated values for the inverse using the tplus operator and return a DQ object.
* To use this member function, type: 'dq_object.pinv();'.
* \return A constant DQ object.
* \sa DQ().
*/
DQ const DQ::pinv() {
    DQ dq;
    DQ pinv;
    DQ tinv;
    for(int n = 0; n < 8; n++) {
        dq.q(n) = q(n);
	}
    //TODO: Generate a message error here if condition is not satisfied
	// Verify if the object caller is a unit DQ
	if (dq.norm() != 1) {
        cout << "ERROR IN PINV OPERATION: NOT A UNIT DUAL QUATERNION \n";
        return dq;
    }

    else {
    // inverse calculation under decompositional multiplication
    tinv = dq.conj();
    tinv = tinv.tplus() * dq.tplus();
    pinv = tinv.conj() * dq.conj();

    // using threshold to verify zero values in DQ to be returned
    for(int n = 0; n < 8; n++) {
        if(fabs(pinv.q(n)) < DQ::threshold() )
            pinv.q(n) = 0;
    }

    return pinv;
    }
};
/**
* Returns a constant DQ object representing the inverse of the unit DQ object caller under decompositional multiplication.
* Actually this function does the same as pinv() changing only the way of calling, which is DQ::pinv(dq_object).
*/
DQ const DQ::pinv(DQ dq) {
 return dq.pinv();
};

/**
* Returns a constant DQ object representing the result of decompositional multiplication between two DQ objects.
*
* To use this member function, type: 'DQ::dec_mult(dq_object1, dq_object2);'.
* \return A constant DQ object.
*/
DQ const DQ::dec_mult(DQ dq1, DQ dq2) {
 DQ dec_mult = dq2.tplus()*dq1.tplus()*dq2.P()*dq1.P();
 return dec_mult;
};

/**
* Returns a constant 4x4 double boost matrix representing the Hamilton operator H+ of primary part of the DQ object caller.
*
* Creates a 4x4 Boost matrix, fill it with values based on the elements q(0) to q(4) and return the matrix H+.
* This operator is applied only for the primary part of DQ. This is the same as consider the DQ object a quaternion.
* To use this member function, type: 'dq_object.Hplus4();'.
* \return A constant boost::numeric::ublas::matrix <double> (4,4).
*/
matrix <double> const DQ::Hplus4() {
    matrix <double> op_Hplus4(4,4);
    op_Hplus4(0,0) = q(0); op_Hplus4(0,1) = -q(1); op_Hplus4(0,2) = -q(2); op_Hplus4(0,3) = -q(3);
    op_Hplus4(1,0) = q(1); op_Hplus4(1,1) =  q(0); op_Hplus4(1,2) = -q(3); op_Hplus4(1,3) =  q(2);
    op_Hplus4(2,0) = q(2); op_Hplus4(2,1) =  q(3); op_Hplus4(2,2) =  q(0); op_Hplus4(2,3) = -q(1);
    op_Hplus4(3,0) = q(3); op_Hplus4(3,1) = -q(2); op_Hplus4(3,2) =  q(1); op_Hplus4(3,3) =  q(0);
    return op_Hplus4;
};
/**
* Returns a constant 4x4 double boost matrix representing the Hamilton operator H+ of primary part of the DQ object caller.
* Actually this function does the same as Hplus4() changing only the way of calling, which is DQ::Hplus4(dq_object).
*/
matrix <double> const DQ::Hplus4(DQ dq) {
 return dq.Hplus4();
};

/**
* Returns a constant 4x4 double boost matrix representing the Hamilton operator H- of primary part of the DQ object caller.
*
* Creates a 4x4 Boost matrix, fill it with values based on the elements q(0) to q(4) and return the matrix H-.
* This operator is applied only for the primary part of DQ. This is the same as consider the DQ object a quaternion.
* To use this member function, type: 'dq_object.Hminus4();'.
* \return A constant boost::numeric::ublas::matrix <double> (4,4).
*/
matrix <double> const DQ::Hminus4() {
    matrix <double> op_Hminus4(4,4);
    op_Hminus4(0,0) = q(0); op_Hminus4(0,1) = -q(1); op_Hminus4(0,2) = -q(2); op_Hminus4(0,3) = -q(3);
    op_Hminus4(1,0) = q(1); op_Hminus4(1,1) =  q(0); op_Hminus4(1,2) =  q(3); op_Hminus4(1,3) = -q(2);
    op_Hminus4(2,0) = q(2); op_Hminus4(2,1) = -q(3); op_Hminus4(2,2) =  q(0); op_Hminus4(2,3) =  q(1);
    op_Hminus4(3,0) = q(3); op_Hminus4(3,1) =  q(2); op_Hminus4(3,2) = -q(1); op_Hminus4(3,3) =  q(0);
    return op_Hminus4;
};
/**
* Returns a constant 4x4 double boost matrix representing the Hamilton operator H- of primary part of the DQ object caller.
* Actually this function does the same as Hminus4() changing only the way of calling, which is DQ::Hminus4(dq_object).
*/
matrix <double> const DQ::Hminus4(DQ dq) {
 return dq.Hminus4();
};

/**
* Returns a constant 8x8 double boost matrix representing the Hamilton operator H+ of the DQ object caller.
*
* Creates a 8x8 Boost matrix, fill it with values based on the elements q(0) to q(8) and return the matrix H+.
* To use this member function, type: 'dq_object.Hplus8();'.
* \return A constant boost::numeric::ublas::matrix <double> (8,8).
*/
matrix <double> const DQ::Hplus8() {
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
/**
* Returns a constant 8x8 double boost matrix representing the Hamilton operator H+ of the DQ object caller.
* * Actually this function does the same as Hplus8() changing only the way of calling, which is DQ::Hplus8(dq_object).
*/
matrix <double> const DQ::Hplus8(DQ dq) {
 return dq.Hplus8();
};

/**
* Returns a constant 8x8 double boost matrix representing the Hamilton operator H- of the DQ object caller.
*
* Creates a 8x8 Boost matrix, fill it with values based on the elements q(0) to q(8) and return the matrix H-.
* To use this member function, type: 'dq_object.Hminus8();'.
* \return A constant boost::numeric::ublas::matrix <double> (8,8).
*/
matrix <double> const DQ::Hminus8() {
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
/**
* Returns a constant 8x8 double boost matrix representing the Hamilton operator H- of the DQ object caller.
* * Actually this function does the same as Hminus8() changing only the way of calling, which is DQ::Hminus8(dq_object).
*/
matrix <double> const DQ::Hminus8(DQ dq) {
 return dq.Hminus8();
};

/**
* Returns a constant 4x1 double Boost matrix representing the 'vec' operator of primary part of the DQ object caller.
*
* Creates a 4x1 Boost matrix, fill it with values based on the elements q(0) to q(4) and return the column matrix vec4.
* This operator is applied only for the primary part of DQ. This is the same as consider the DQ object a quaternion.
* To use this member function, type: 'dq_object.vec4();'.
* \return A constant boost::numeric::ublas::matrix <double> (4,1).
*/
matrix <double> const DQ::vec4() {
    matrix <double> op_vec4(4,1);
    op_vec4(0,0) = q(0);
    op_vec4(1,0) = q(1);
    op_vec4(2,0) = q(2);
    op_vec4(3,0) = q(3);
    return op_vec4;
};
/**
* Returns a constant 4x1 double Boost matrix representing the 'vec' operator of primary part of the DQ object caller.
* Actually this function does the same as vec4() changing only the way of calling, which is DQ::vec4(dq_object).
*/
matrix <double> const DQ::vec4(DQ dq) {
    return dq.vec4();
};

/**
* Returns a constant 8x1 double boost matrix representing the 'vec' operator of the DQ object caller.
*
* Creates a 8x1 Boost matrix, fill it with values based on the elements q(0) to q(8) and return the column matrix vec8.
* To use this member function, type: 'dq_object.vec8();'.
* \return A constant boost::numeric::ublas::matrix <double> (8,1).
*/
matrix <double> const DQ::vec8() {
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
/**
* Returns a constant 8x1 double boost matrix representing the 'vec' operator of the DQ object caller.
* Actually this function does the same as vec8() changing only the way of calling, which is DQ::vec8(dq_object).
*/
matrix <double> const DQ::vec8(DQ dq) {
 return dq.vec8();
};

/** Returns the Generilized Jacobian; that it, the Jacobian that satisfies the relation Geometric_Jacobian = G * DQ_Jacobian.
* To use this member function type: 'dq_object.jacobG(x_E).
* \param DQ x_E is the dual position quaternion
* \return A constant boost::numeric::ublas::matrix <double>
*/
matrix <double> const DQ::jacobG(DQ x_E) {
    matrix <double> jacobGen(8,8);
    jacobGen(0,0) = x_E.q(4); jacobGen(0,1) =  x_E.q(5); jacobGen(0,2) =  x_E.q(6); jacobGen(0,3) =  x_E.q(7);
    jacobGen(1,0) = x_E.q(5); jacobGen(1,1) = -x_E.q(4); jacobGen(1,2) =  x_E.q(7); jacobGen(1,3) = -x_E.q(6);
    jacobGen(2,0) = x_E.q(6); jacobGen(2,1) = -x_E.q(7); jacobGen(2,2) = -x_E.q(4); jacobGen(2,3) =  x_E.q(5);
    jacobGen(3,0) = x_E.q(7); jacobGen(3,1) =  x_E.q(6); jacobGen(3,2) = -x_E.q(5); jacobGen(3,3) = -x_E.q(4);

    jacobGen(0,4) =  x_E.q(0); jacobGen(0,5) =  x_E.q(1); jacobGen(0,6) =  x_E.q(2); jacobGen(0,7) =  x_E.q(3);
    jacobGen(1,4) = -x_E.q(1); jacobGen(1,5) =  x_E.q(0); jacobGen(1,6) = -x_E.q(3); jacobGen(1,7) =  x_E.q(2);
    jacobGen(2,4) = -x_E.q(2); jacobGen(2,5) =  x_E.q(3); jacobGen(2,6) =  x_E.q(0); jacobGen(2,7) = -x_E.q(1);
    jacobGen(3,4) = -x_E.q(3); jacobGen(3,5) = -x_E.q(2); jacobGen(3,6) =  x_E.q(1); jacobGen(3,7) =  x_E.q(0);

    jacobGen(4,0) =  x_E.q(0); jacobGen(4,1) =  x_E.q(1); jacobGen(4,2) =  x_E.q(2); jacobGen(4,3) =  x_E.q(3);
    jacobGen(5,0) = -x_E.q(1); jacobGen(5,1) =  x_E.q(0); jacobGen(5,2) = -x_E.q(3); jacobGen(5,3) =  x_E.q(2);
    jacobGen(6,0) = -x_E.q(2); jacobGen(6,1) =  x_E.q(3); jacobGen(6,2) =  x_E.q(0); jacobGen(6,3) = -x_E.q(1);
    jacobGen(7,0) = -x_E.q(3); jacobGen(7,1) = -x_E.q(2); jacobGen(7,2) =  x_E.q(1); jacobGen(7,3) =  x_E.q(0);

    jacobGen(4,4) = 0; jacobGen(4,5) = 0; jacobGen(4,6) = 0; jacobGen(4,7) = 0;
    jacobGen(5,4) = 0; jacobGen(5,5) = 0; jacobGen(5,6) = 0; jacobGen(5,7) = 0;
    jacobGen(6,4) = 0; jacobGen(6,5) = 0; jacobGen(6,6) = 0; jacobGen(6,7) = 0;
    jacobGen(7,4) = 0; jacobGen(7,5) = 0; jacobGen(7,6) = 0; jacobGen(7,7) = 0;

    return 2*jacobGen;
};

/** Returns the Generilized Jacobian; that it, the Jacobian that satisfies the relation Geometric_Jacobian = G * DQ_Jacobian.
* Actually this function does the same as jacobG(theta_vec) changing only the way of calling, which is
* DQ::jacobG(dq_object, x_E).
*/
matrix <double> const DQ::jacobG(DQ param_dq, DQ x_E) {
    return param_dq.jacobG(x_E);
};

/**
* Display DQ object caller.
*
* Concatenate the strings generated by the build_string function with other string elements to display correctly, the DQ object caller
* including it's object name, the primary and dual parts. The DQ object is displayed in the following standard form:
* dq_object = (q(0) + q(1)*i + q(2)*j + q(3)*k) + dual_unit_E*(q(4) + q(5)*i + q(6)*j + q(7)*k). To use this member funtion correctly
* and comfortably, type DISPLAY(dq_object). Where 'DISPLAY" must be upper case because it's a defined macro created to facilitate the use.
* \param name is the name of the DQ object passed to function (using DISPLAY macro this could be ignored)
* \param dq is the dual quaternion to be displayed.
*/
void DQ::display(char *name, DQ dq) {

	has_primary_element = 0;
	has_dual_element = 0;
    std::string s, sd;

    s = DQ::build_string(dq,0);
    sd = DQ::build_string(dq,4);

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
    cout << name << " = " << s << sd << "\n";
};

/**
* Display a boost matrix representing Hamilton or vec operators.
*
* This function just displays the boost matrix in a good form of visualisation. To use this member funtion correctly and comfortably,
* type MATRIX(matrix <double> H_or_vec). Where 'MATRIX" must be upper case because it's a defined macro created to facilitate the use.
* \param name is the name of the matrix passed to function (using MATRIX macro this could be ignored)
* \param &H_or_vec is a reference to the matrix to be displayed.
*/
void DQ::display(char *name, matrix <double> &H_or_vec) {

    cout << name << " = \n";
    for(unsigned int line = 0; line < H_or_vec.size1() ; line++) {
        for(unsigned int column = 0; column < H_or_vec.size2() ; column++) {
            if(fabs(H_or_vec(line, column)) < DQ::threshold() )
            H_or_vec(line, column) = 0;
            cout << "\t" << std::setw(10) << std::left << H_or_vec(line, column);
        }
        cout << "\n";
    }
    cout << "\n";
};

/**
* Display a boost vector as a boost matrix 1xn.
*
* This function just displays the boost vector in a good form of visualisation. To use this member funtion correctly and comfortably,
* type MATRIX(vector <double> vec). Where 'MATRIX" must be upper case because it's a defined macro created to facilitate the use.
* \param name is the name of the vector passed to function (using MATRIX macro this could be ignored)
* \param &vec is a reference to the vector to be displayed.
*/
void DQ::display(char *name, vector <double> &vec) {

    cout << name << " = \n";
    matrix <double> vec_line (1,vec.size());
    for(unsigned int col = 0; col < vec.size() ; col++) {
	vec_line(0,col) = vec(col);
    }
    for(unsigned int line = 0; line < vec_line.size1() ; line++) {
        for(unsigned int column = 0; column < vec_line.size2() ; column++) {
            if(fabs(vec_line(line, column)) < DQ::threshold() )
            vec_line(line, column) = 0;
            cout << "\t" << std::setw(10) << std::left << vec_line(line, column);
        }
        cout << "\n";
    }
    cout << "\n";
};

/**
* Unit Dual Quaternion constructor.
*
* Returns a DQ object with unitary norm defined as DQ h = r + DQ::E(r)*0.5*p*r. Where, r is a rotation quatérnion composed by a
* rotation angle and rotation axis. and p is a translation quatérnion composed by three displacements on the three coordinates x,y and z.
* To create a DQ object using this, type: 'unit_DQ_h = DQ::unitDQ(PI/4, 0,0,1, 2.2,0,2.457);' for example.
* \param rot_angle is the angle rotation of the unit DQ.
* \param x_axis, y_axis and z_axis are the axis which are rotated
* \param x_trans, y_trans and z_trans are the displacements of translation on the respective axis x, y and/or z.
* \return A DQ object.
*/
DQ DQ::unitDQ(double rot_angle, int x_axis,int y_axis,int z_axis, double x_trans,double y_trans, double z_trans) {
    if ((x_axis != 0 && x_axis != 1) || (y_axis != 0 && y_axis != 1) || (z_axis != 0 && z_axis != 1)) {
        cout << "ERROR IN ROTATION ANGLE CHOOSE: X, Y AND Z AXIS PARAMETERS MUST BE 1 OR 0 \n";
        return DQ(0);
    }
    vector <double> axis(4), translation(4);
    axis(0) = 0;
    axis(1) = x_axis;
    axis(2) = y_axis;
    axis(3) = z_axis;

    translation(0) = 0;
    translation(1) = x_trans;
    translation(2) = y_trans;
    translation(3) = z_trans;

    DQ r = cos(rot_angle/2) + sin(rot_angle/2)*DQ(axis);
    DQ p = DQ(translation);
    DQ h = r + DQ::E(r)*0.5*p*r;

    // using threshold to verify zero values in DQ to be returned
    for(int n = 0; n < 8; n++) {
        if(fabs(h.q(n)) < DQ::threshold() )
            h.q(n) = 0;
    }
    return h;
};

//Private auxiliar functions

/**
* Constructs and Returns a string to be displayed representing a part of the DQ object caller.
*
* Creates a string used by the display() member function which contains, parentesis, the imaginary units, and the elements scalar
* values of the primary or dual part of the dual quaternion. The part being constructed is defined by a shift variable.
* \param dq is the dual quaternion to be displayed.
* \param shift is an integer that shifts the focus to the part which the display string is being constructed (primary or dual).
* \return A std::string.
* \sa display().
*/
std::string DQ::build_string(DQ dq, int shift) {
    std::string s, aux;
    char buffer_number[32];
    s = "";
    aux = " ijk";

    int has_element = 0;

    for(int n = 0; n < 4; n++) {
        if (fabs(dq.q(n+shift) ) > dq.threshold() ) { //To avoid printing values very close to zero
            if (dq.q(n+shift) < 0)
                s = s + " - ";
            else if (has_element != 0)
                s = s + " + ";

            if (n == 0) {
                sprintf(buffer_number, "%1.5g", fabs(dq.q(n+shift) ) ); //coverts DQ element to string
                s = s + buffer_number; //concatenate the real number to string generated yet
            }
            else {
                sprintf(buffer_number, "%1.5g", fabs(dq.q(n+shift) ) ); //coverts DQ element to string
                s = s + buffer_number + aux[n]; //concatenate the imaginary number to string generated yet
            }

            if (shift == 0) { //mounting primary part
            has_primary_element = 1;
            has_element = 1;
            }
            else { //mounting dual part
            has_dual_element = 1;
            has_element = 1;
            }
        }
    }
    return s;
};

/**
* Returns a scalar value to be used as a threshold in some internal operations
*
* Creates a double type constant with the value 0.000000000001 or 10e-12. In several definitions inside the class member functions
* this number is used to verify if a value should be considered zero or not.
* \return A const double value.
*/
double DQ::threshold() {
	const double threshold = 0.000000000001;
	return threshold;
};

//Overloaded operators definitions

/**
* Operator (+) overload for the sum of two DQ objects.
*
* This friend function realizes the sum of two DQ objects and returns the result on another DQ object which is created with default
* constructor and have the vector 'q' modified acording to the operation.
* \param dq1 is the first DQ object in the operation.
* \param dq2 is the second DQ object in the operation.
* \return A DQ object.
* \sa DQ(), threshold().
*/
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


/**
* Operator (+) overload for the sum of a DQ object and an integer scalar
*
* This friend function realizes the sum of a DQ object and an integer scalar and returns the result on another DQ object.
* A DQ object is created by the DQ constructor using a scalar element and then the operation is made between two DQ objects.
* \param dq is the DQ object in the operation.
* \param scalar is an integer scalar involved in operation.
* \return A DQ object.
* \sa DQ(double scalar), operator+(DQ dq1, DQ dq2).
*/
DQ operator+(DQ dq, int scalar) {
    DQ dq_scalar(scalar);
    return (dq + dq_scalar);
};

/**
* Operator (+) overload for the sum of an integer scalar and DQ object
*
* This friend function realizes the sum of an integer scalar and a DQ object and returns the result on another DQ object.
* A DQ object is created by the DQ constructor using a scalar element and then the operation is made between two DQ objects.
* \param scalar is an integer scalar involved in operation.
* \param dq is the DQ object in the operation.
* \return A DQ object.
* \sa DQ(double scalar), operator+(DQ dq1, DQ dq2).
*/
DQ operator+(int scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar + dq);
};

/**
* Operator (+) overload for the sum of a DQ object and a float scalar
*
* This friend function realizes the sum of a DQ object and a float scalar and returns the result on another DQ object.
* A DQ object is created by the DQ constructor using a scalar element and then the operation is made between two DQ objects.
* \param dq is the DQ object in the operation.
* \param scalar is a float scalar involved in operation.
* \return A DQ object.
* \sa DQ(double scalar), operator+(DQ dq1, DQ dq2).
*/
DQ operator+(DQ dq, float scalar) {
    DQ dq_scalar(scalar);
    return (dq + dq_scalar);
};

/**
* Operator (+) overload for the sum of a float scalar and DQ object
*
* This friend function realizes the sum of a float scalar and a DQ object and returns the result on another DQ object.
* A DQ object is created by the DQ constructor using a scalar element and then the operation is made between two DQ objects.
* \param scalar is a float scalar involved in operation.
* \param dq is the DQ object in the operation.
* \return A DQ object.
* \sa DQ(double scalar), operator+(DQ dq1, DQ dq2).
*/
DQ operator+(float scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar + dq);
};

/**
* Operator (+) overload for the sum of a DQ object and a double scalar
*
* This friend function realizes the sum of a DQ object and a double scalar and returns the result on another DQ object.
* A DQ object is created by the DQ constructor using a scalar element and then the operation is made between two DQ objects.
* \param dq is the DQ object in the operation.
* \param scalar is a double scalar involved in operation.
* \return A DQ object.
* \sa DQ(double scalar), operator+(DQ dq1, DQ dq2).
*/
DQ operator+(DQ dq, double scalar) {
    DQ dq_scalar(scalar);
    return (dq + dq_scalar);
};

/**
* Operator (+) overload for the sum of a double scalar and DQ object
*
* This friend function realizes the sum of a double scalar and a DQ object and returns the result on another DQ object.
* A DQ object is created by the DQ constructor using a scalar element and then the operation is made between two DQ objects.
* \param scalar is a double scalar involved in operation.
* \param dq is the DQ object in the operation.
* \return A DQ object.
* \sa DQ(double scalar), operator+(DQ dq1, DQ dq2).
*/
DQ operator+(double scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar + dq);
};

// Operator (-) overload

/**
* Operator (-) overload to subtract one DQ object of other.
*
* This friend function do the subtraction of a DQ object in other and returns the result on another DQ object which
* is created with default constructor and have the vector 'q' modified acording to the operation.
* \param dq1 is the first DQ object in the operation.
* \param dq2 is the second DQ object in the operation.
* \return A DQ object.
* \sa DQ(), threshold().
*/
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

/**
* Operator (-) overload to subtract an integer scalar of one DQ object.
*
* This friend function realizes the subtraction of a integer scalar in one DQ object. and returns the result on another DQ object.
* A DQ object is created by the DQ constructor using a scalar element and then the operation is made between two DQ objects.
* \param dq is the DQ object in the operation.
* \param scalar is the integer scalar involved in operation.
* \return A DQ object.
* \sa DQ(double scalar), operator-(DQ dq1, DQ dq2).
*/
DQ operator-(DQ dq, int scalar) {
    DQ dq_scalar(scalar);
    return (dq - dq_scalar);
};

/**
* Operator (-) overload to subtract a DQ object of one integer scalar.
*
* This friend function realizes the subtraction of a DQ object in one integer scalar. and returns the result on another DQ object.
* A DQ object is created by the DQ constructor using a scalar element and then the operation is made between two DQ objects.
* \param scalar is the integer scalar involved in operation.
* \param dq is the DQ object in the operation.
* \return A DQ object.
* \sa DQ(double scalar), operator-(DQ dq1, DQ dq2).
*/
DQ operator-(int scalar, DQ dq){
    DQ dq_scalar(scalar);
    return (dq_scalar - dq);
};

/**
* Operator (-) overload to subtract an float scalar of one DQ object.
*
* This friend function realizes the subtraction of a float scalar in one DQ object. and returns the result on another DQ object.
* A DQ object is created by the DQ constructor using a scalar element and then the operation is made between two DQ objects.
* \param dq is the DQ object in the operation.
* \param scalar is the float scalar involved in operation.
* \return A DQ object.
* \sa DQ(double scalar), operator-(DQ dq1, DQ dq2).
*/
DQ operator-(DQ dq, float scalar){
    DQ dq_scalar(scalar);
    return (dq - dq_scalar);
};

/**
* Operator (-) overload to subtract a DQ object of one float scalar.
*
* This friend function realizes the subtraction of a DQ object in one float scalar. and returns the result on another DQ object.
* A DQ object is created by the DQ constructor using a scalar element and then the operation is made between two DQ objects.
* \param scalar is the float scalar involved in operation.
* \param dq is the DQ object in the operation.
* \return A DQ object.
* \sa DQ(double scalar), operator-(DQ dq1, DQ dq2).
*/
DQ operator-(float scalar, DQ dq){
    DQ dq_scalar(scalar);
    return (dq_scalar - dq);
};

/**
* Operator (-) overload to subtract an double scalar of one DQ object.
*
* This friend function realizes the subtraction of a double scalar in one DQ object. and returns the result on another DQ object.
* A DQ object is created by the DQ constructor using a scalar element and then the operation is made between two DQ objects.
* \param dq is the DQ object in the operation.
* \param scalar is the double scalar involved in operation.
* \return A DQ object.
* \sa DQ(double scalar), operator-(DQ dq1, DQ dq2).
*/
DQ operator-(DQ dq, double scalar){
    DQ dq_scalar(scalar);
    return (dq - dq_scalar);
};

/**
* Operator (-) overload to subtract a DQ object of one double scalar.
*
* This friend function realizes the subtraction of a DQ object in one double scalar. and returns the result on another DQ object.
* A DQ object is created by the DQ constructor using a scalar element and then the operation is made between two DQ objects.
* \param scalar is the double scalar involved in operation.
* \param dq is the DQ object in the operation.
* \return A DQ object.
* \sa DQ(double scalar), operator-(DQ dq1, DQ dq2).
*/
DQ operator-(double scalar, DQ dq){
    DQ dq_scalar(scalar);
    return (dq_scalar - dq);
};

// Operator (*) overload

/**
* Operator (*) overload for the standard multiplication of two DQ objects.
*
* This friend function do the standard multiplication of two DQ objects and returns the result on another DQ object which
* is created with default constructor and have the vector 'q' modified acording to the operation.
* \param dq1 is the first DQ object in the operation.
* \param dq2 is the second DQ object in the operation.
* \return A DQ object.
* \sa DQ(), D(), P(), threshold().
*/
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

/**
* Operator (*) overload for the multiplication of a DQ object and an integer scalar
*
* This friend function realizes the multiplication of a DQ object and an integer scalar and returns the result on another DQ object.
* A DQ object is created by the DQ constructor using a scalar element and then the operation is made between two DQ objects.
* \param dq is the DQ object in the operation.
* \param scalar is an integer scalar involved in operation.
* \return A DQ object.
* \sa DQ(double scalar), operator*(DQ dq1, DQ dq2).
*/
DQ operator*(DQ dq, int scalar) {
    DQ dq_scalar(scalar);
    return (dq * dq_scalar);
};

/**
* Operator (*) overload for the multiplication of an integer scalar and DQ object
*
* This friend function realizes the multiplication of an integer scalar and a DQ object and returns the result on another DQ object.
* A DQ object is created by the DQ constructor using a scalar element and then the operation is made between two DQ objects.
* \param scalar is an integer scalar involved in operation.
* \param dq is the DQ object in the operation.
* \return A DQ object.
* \sa DQ(double scalar), operator*(DQ dq1, DQ dq2).
*/
DQ operator*(int scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar * dq);
};

/**
* Operator (*) overload for the multiplication of a DQ object and a float scalar
*
* This friend function realizes the multiplication of a DQ object and a float scalar and returns the result on another DQ object.
* A DQ object is created by the DQ constructor using a scalar element and then the operation is made between two DQ objects.
* \param dq is the DQ object in the operation.
* \param scalar is a float scalar involved in operation.
* \return A DQ object.
* \sa DQ(double scalar), operator*(DQ dq1, DQ dq2).
*/
DQ operator*(DQ dq, float scalar) {
    DQ dq_scalar(scalar);
    return (dq * dq_scalar);
};

/**
* Operator (*) overload for the multiplication of a float scalar and DQ object
*
* This friend function realizes the multiplication of a float scalar and a DQ object and returns the result on another DQ object.
* A DQ object is created by the DQ constructor using a scalar element and then the operation is made between two DQ objects.
* \param scalar is a float scalar involved in operation.
* \param dq is the DQ object in the operation.
* \return A DQ object.
* \sa DQ(double scalar), operator*(DQ dq1, DQ dq2).
*/
DQ operator*(float scalar, DQ dq){
    DQ dq_scalar(scalar);
    return (dq_scalar * dq);
};

/**
* Operator (*) overload for the multiplication of a DQ object and an double scalar
*
* This friend function realizes the multiplication of a DQ object and an double scalar and returns the result on another DQ object.
* A DQ object is created by the DQ constructor using a scalar element and then the operation is made between two DQ objects.
* \param dq is the DQ object in the operation.
* \param scalar is an double scalar involved in operation.
* \return A DQ object.
* \sa DQ(double scalar), operator*(DQ dq1, DQ dq2).
*/
DQ operator*(DQ dq, double scalar){
    DQ dq_scalar(scalar);
    return (dq * dq_scalar);
};

/**
* Operator (*) overload for the multiplication of an double scalar and DQ object
*
* This friend function realizes the multiplication of an double scalar and a DQ object and returns the result on another DQ object.
* A DQ object is created by the DQ constructor using a scalar element and then the operation is made between two DQ objects.
* \param scalar is an double scalar involved in operation.
* \param dq is the DQ object in the operation.
* \return A DQ object.
* \sa DQ(double scalar), operator*(DQ dq1, DQ dq2).
*/
DQ operator*(double scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar * dq);
};

// Operator (==) overload

/**
* Operator (==) overload for the comparison between two DQ objects.
*
* This function do the comparison of two DQ objects. One is the DQ object caller, the first member in operation.
* The result is returned as a boolean variable. True, means that both DQ objects are equal.
* \param dq2 is the second DQ object in the operation.
* \return A boolean variable.
* \sa threshold().
*/
bool DQ::operator==(DQ dq2) {
    for(int n = 0; n<8; n++) {
        if(fabs(q(n) - dq2.q(n)) > DQ::threshold() )
        return false; //elements of Dual Quaternion different of scalar
    }
    return true; //elements of Dual Quaternion equal to scalar
};

/**
* Operator (==) overload for the comparison between a DQ object and an integer scalar.
*
* This function do the comparison between a DQ object and an integer scalar. The scalar is transformed in a DQ object and then
* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that both
* DQ objects are equal and thus the DQ object is equal to the scalar.
* \param dq is the DQ object in the operation.
* \param scalar is an integer scalar involved in operation
* \return A boolean variable.
* \sa DQ(double scalar), operator==(DQ dq2).
*/
bool operator==(DQ dq, int scalar) {
    DQ dq_scalar(scalar);
    return (dq == dq_scalar);
};

/**
* Operator (==) overload for the comparison between an integer scalar and a DQ object
*
* This function do the comparison between a DQ object and an integer scalar. The scalar is transformed in a DQ object and then
* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that both
* DQ objects are equal and thus the DQ object is equal to the scalar.
* \param scalar is an integer scalar involved in operation
* \param dq is the DQ object in the operation.
* \return A boolean variable.
* \sa DQ(double scalar), operator==(DQ dq2).
*/
bool operator==(int scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar == dq);
};

/**
* Operator (==) overload for the comparison between a DQ object and a float scalar.
*
* This function do the comparison between a DQ object and a float scalar. The scalar is transformed in a DQ object and then
* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that both
* DQ objects are equal and thus the DQ object is equal to the scalar.
* \param dq is the DQ object in the operation.
* \param scalar is a float scalar involved in operation
* \return A boolean variable.
* \sa DQ(double scalar), operator==(DQ dq2).
*/
bool operator==(DQ dq, float scalar) {
    DQ dq_scalar(scalar);
    return (dq == dq_scalar);
};

/**
* Operator (==) overload for the comparison between a float scalar and a DQ object
*
* This function do the comparison between a DQ object and a float scalar. The scalar is transformed in a DQ object and then
* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that both
* DQ objects are equal and thus the DQ object is equal to the scalar.
* \param scalar is a float scalar involved in operation
* \param dq is the DQ object in the operation.
* \return A boolean variable.
* \sa DQ(double scalar), operator==(DQ dq2).
*/
bool operator==(float scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar == dq);
};

/**
* Operator (==) overload for the comparison between a DQ object and a double scalar.
*
* This function do the comparison between a DQ object and a double scalar. The scalar is transformed in a DQ object and then
* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that both
* DQ objects are equal and thus the DQ object is equal to the scalar.
* \param dq is the DQ object in the operation.
* \param scalar is an double scalar involved in operation
* \return A boolean variable.
* \sa DQ(double scalar), operator==(DQ dq2).
*/
bool operator==(DQ dq, double scalar) {
    DQ dq_scalar(scalar);
    return (dq == dq_scalar);
};

/**
* Operator (==) overload for the comparison between a double scalar and a DQ object
*
* This function do the comparison between a DQ object and a double scalar. The scalar is transformed in a DQ object and then
* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that both
* DQ objects are equal and thus the DQ object is equal to the scalar.
* \param scalar is a double scalar involved in operation
* \param dq is the DQ object in the operation.
* \return A boolean variable.
* \sa DQ(double scalar), operator==(DQ dq2).
*/
bool operator==(double scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar == dq);
};

// Operator (!=) overload

/**
* Operator (!=) overload for the comparison between two DQ objects.
*
* This function do the comparison of two DQ objects. One is the DQ object caller, the first member in operation.
* The result is returned as a boolean variable. True, means that DQ objects are not equal.
* \param dq2 is the second DQ object in the operation.
* \return A boolean variable.
* \sa threshold().
*/
bool DQ::operator!=(DQ dq2) {
    for(int n = 0; n<8; n++){
        if(fabs(q(n) - dq2.q(n)) > DQ::threshold() )
        return true; //elements of Dual Quaternion different of scalar
    }
    return false; //elements of Dual Quaternion equal to scalar
};

/**
* Operator (!=) overload for the comparison between a DQ object and an integer scalar.
*
* This friend function do the comparison between a DQ object and an integer scalar. The scalar is transformed in a DQ object and then
* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that DQ objects
* are not equal and thus the DQ object isn't equal to the scalar.
* \param dq is the DQ object in the operation.
* \param scalar is an integer scalar involved in operation
* \return A boolean variable.
* \sa DQ(double scalar), operator!=(DQ dq2).
*/
bool operator!=(DQ dq, int scalar) {
    DQ dq_scalar(scalar);
    return (dq != dq_scalar);
};

/**
* Operator (!=) overload for the comparison between an integer scalar and a DQ object
*
* This function do the comparison between a DQ object and an integer scalar. The scalar is transformed in a DQ object and then
* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that DQ objects
* are not equal and thus the DQ object isn't equal to the scalar.
* \param scalar is an integer scalar involved in operation
* \param dq is the DQ object in the operation.
* \return A boolean variable.
* \sa DQ(double scalar), operator!=(DQ dq2).
*/
bool operator!=(int scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar != dq);
};

/**
* Operator (!=) overload for the comparison between a DQ object and a float scalar.
*
* This friend function do the comparison between a DQ object and a float scalar. The scalar is transformed in a DQ object and then
* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that DQ objects
* are not equal and thus the DQ object isn't equal to the scalar.
* \param dq is the DQ object in the operation.
* \param scalar is a float scalar involved in operation
* \return A boolean variable.
* \sa DQ(double scalar), operator!=(DQ dq2).
*/
bool operator!=(DQ dq, float scalar) {
    DQ dq_scalar(scalar);
    return (dq != dq_scalar);
};

/**
* Operator (!=) overload for the comparison between a float scalar and a DQ object
*
* This function do the comparison between a DQ object and a float scalar. The scalar is transformed in a DQ object and then
* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that DQ objects
* are not equal and thus the DQ object isn't equal to the scalar.
* \param scalar is a float scalar involved in operation
* \param dq is the DQ object in the operation.
* \return A boolean variable.
* \sa DQ(double scalar), operator!=(DQ dq2).
*/
bool operator!=(float scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar != dq);
};

/**
* Operator (!=) overload for the comparison between a DQ object and a double scalar.
*
* This friend function do the comparison between a DQ object and a double scalar. The scalar is transformed in a DQ object and then
* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that DQ objects
* are not equal and thus the DQ object isn't equal to the scalar.
* \param dq is the DQ object in the operation.
* \param scalar is a double scalar involved in operation
* \return A boolean variable.
* \sa DQ(double scalar), operator!=(DQ dq2).
*/
bool operator!=(DQ dq, double scalar) {
    DQ dq_scalar(scalar);
    return (dq != dq_scalar);
};

/**
* Operator (!=) overload for the comparison between a double scalar and a DQ object
*
* This function do the comparison between a DQ object and a double scalar. The scalar is transformed in a DQ object and then
* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that DQ objects
* are not equal and thus the DQ object isn't equal to the scalar.
* \param scalar is a double scalar involved in operation
* \param dq is the DQ object in the operation.
* \return A boolean variable.
* \sa DQ(double scalar), operator!=(DQ dq2).
*/
bool operator!=(double scalar, DQ dq) {
    DQ dq_scalar(scalar);
    return (dq_scalar != dq);
};

// Operator (^) overload

/**
* Operator (^) overload for the potentiation operation of a DQ objects.
*
* This friend function do the raises a DQ object by the m-th power and returns the result on another DQ object which
* elements are correctly calculated
* \param dq1 is the DQ object in the operation.
* \param m is the double value representing the power.
* \return A DQ object.
* \sa DQ(), log(), exp() threshold().
*/
DQ operator^(DQ dq1, double m) {
    //TODO: Generate a message error here if condition is not satisfied
    if (dq1.norm() != 1) {
        cout << "ERROR IN MPOWER OPERATION: NOT A UNIT DUAL QUATERNION \n";
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
