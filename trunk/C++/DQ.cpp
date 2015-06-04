/**
(C) Copyright 2015 DQ Robotics Developers

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

#include "DQ.h"

using std::cout;

namespace DQ_robotics{


/****************************************************************
**************NAMESPACE ONLY FUNCTIONS***************************
*****************************************************************/

/**
* P() operator -> retrieves the primary part of a DQ. 
*
* @param dq The DQ which primary part you wish.
* @return a constant DQ representing the primary part of dq.
*/
DQ P(const DQ& dq)
{
    return dq.P();
}

/**
* D() operator -> retrieves the dual part of a DQ. 
*
* @param dq The DQ which dual part you wish.
* @return a constant DQ representing the dual part of dq.
*/
DQ D(const DQ& dq)
{
    return dq.D();
}

/**
* Re() operator -> retrieves the real part of a DQ. 
*
* @param dq The DQ which real part you wish.
* @return a constant DQ representing the real part of dq.
*/
DQ Re(const DQ& dq)
{
    return dq.Re();
}

/**
* Im() operator -> retrieves the imaginary part of a DQ. 
*
* @param dq The DQ which imaginary part you wish.
* @return a constant DQ representing the imaginary part of dq.
*/
DQ Im(const DQ& dq)
{
    return dq.Im();
}

/**
* Conjugate operator -> retrieves the conjugate of a DQ. 
*
* @param dq The DQ which conjugate you wish.
* @return a constant DQ representing the conjugate of dq.
*/
DQ conj(const DQ& dq)
{
    return dq.conj();
}

/**
* Norm operator -> retrieves the norm of a DQ. 
*
* @param dq The DQ which norm you wish.
* @return a constant DQ representing the norm of dq.
*/
DQ norm(const DQ& dq)
{
    return dq.norm();
}

/**
* Norm operator -> retrieves the norm of a DQ. 
*
* @param dq The DQ which norm you wish.
* @return a constant DQ representing the norm of dq.
*/
DQ inv(const DQ& dq)
{
    return dq.inv();
}

/**
* Translation operator -> retrieves the Translation represented by a DQ. 
*
* @param dq The DQ which Translation you wish.
* @return a constant DQ representing the Translation represented by dq.
*/
DQ translation(const DQ& dq)
{
    return dq.translation();
}

/**
* Rotation Axis operator -> retrieves the Rotation Axis represented by a DQ. 
*
* @param dq The DQ which Rotation Axis you wish.
* @return a constant DQ representing the Rotation Axis represented by dq.
*/
DQ rot_axis(const DQ& dq)
{
    return dq.rot_axis();
}

/**
* Rotation Angle operator -> retrieves the Rotation Angle represented by a DQ. 
*
* @param dq The DQ which Rotation Angle you wish.
* @return a constant DQ representing the Rotation Angle represented by dq.
*/
double rot_angle(const DQ& dq)
{
    return dq.rot_angle();
}

/**
* Logarithm operator -> retrieves the Logarithm of a DQ. 
*
* @param dq The DQ which Logarithm you wish.
* @return a constant DQ representing the Logarithm of dq.
*/
DQ log(const DQ& dq)
{
    return dq.log();
}

/**
* Exponential operator -> retrieves the Exponential of a DQ. 
*
* @param dq The DQ which Exponential you wish.
* @return a constant DQ representing the Exponential of dq.
*/
DQ exp(const DQ& dq)
{
    return dq.exp();
}

/**
* Exponential operator -> retrieves the Exponential of a DQ. 
*
* @param dq The DQ which Exponential you wish.
* @return a constant DQ representing the Exponential of dq.
*/
DQ tplus(const DQ& dq)
{
    return dq.tplus();
}

/**
* Inverse of a unit DQ under decompositional multiplication.
*
* @param dq The DQ which inverse you wish.
* @return a constant DQ representing the inverse of dq.
*/
DQ pinv(const DQ& dq)
{
    return dq.pinv();
}

/**
* Decompositional multiplication.
*
* @param dq1 a DQ.
* @param dq2 a DQ.
* @return Returns a constant DQ object representing the result of decompositional multiplication between two dq1 and dq2.
*/
DQ dec_mult(const DQ& dq1, const DQ& dq2)
{
    DQ dec_mult = dq2.tplus()*dq1.tplus()*dq2.P()*dq1.P();
    return dec_mult;
}


/**
* Hamilton operator H+ for the primary part.
*
* @param dq The DQ which H+ you wish.
* @return the 4x4 matrix representing the H+ operator of the primary part of dq.
*/
Matrix4d Hplus4(const DQ& dq)
{
    return dq.Hplus4();
}

/**
* Hamilton operator H- for the primary part.
*
* @param dq The DQ which H- you wish.
* @return the 4x4 matrix representing the H- operator of the primary part of dq.
*/
Matrix4d Hminus4(const DQ& dq)
{
    return dq.Hminus4();
}

/**
* Hamilton operator H+.
*
* @param dq The DQ which H+ you wish.
* @return the 8x8 matrix representing the H+ operator of dq.
*/
Matrix<double,8,8> Hplus8(const DQ& dq)
{
    return dq.Hplus8();
}

/**
* Hamilton operator H-.
*
* @param dq The DQ which H- you wish.
* @return the 8x8 matrix representing the H- operator of dq.
*/
Matrix<double,8,8> Hminus8(const DQ& dq)
{
    return dq.Hminus8();
}

/**
* Vect operator on the primary part of a DQ.
*
* @param dq The DQ with the primary part you wish to obtain the vect of. 
* @return A 4x1 vector representing the primary part of dq.
*/
Vector4d vec4(const DQ& dq)
{
    return dq.vec4();
}

/**
* Vect operator on a DQ.
*
* @param dq The DQ in which you want to apply the operation. 
* @return A 8x1 vector representing dq.
*/
Matrix<double,8,1>  vec8(const DQ& dq)
{
    return dq.vec8();
}

/**
* Generalized Jacobian, i.e, the Jacobian that satisfies the relation Geometric Jacobian = Generalized Jacobian * Analytical (DQ) Jacobian.
*
* @param param_dq.
* @param x_e.
* @return A 8x8 matrix representing the Generalized Jacobian given the arguments.
*/
Matrix<double,8,8> jacobG(const DQ& x_E)
{
    return x_E.jacobG();
}

/**
* Generalized Jacobian, i.e, the Jacobian that satisfies the relation Geometric Jacobian = Generalized Jacobian * Analytical (DQ) Jacobian.
*
* @param param_dq.
* @param x_e.
* @return A 8x8 matrix representing the Generalized Jacobian given the arguments.
*/
Matrix<double,8,8> generalizedJacobian(const DQ& x_E)
{
    return x_E.jacobG();
}

/****************************************************************
**************DQ CLASS METHODS***********************************
*****************************************************************/


/**
* DQ constructor using boost vector
*
* Returns a DQ object with the values of elements equal to the values of elements from a vector 'v' passed to constructor.
* \param vector <double> v contain the values to copied to the attribute q.
*/
DQ::DQ( const Matrix<double,8,1>& v) {
    q = v;
}


/**
* DQ constructor using boost vector
*
* Returns a DQ object with the values of elements equal to the values of elements from a vector 'v' passed to constructor.
* \param vector <double> v contain the values to copied to the attribute q.
*/
DQ::DQ( const Matrix<double,4,1>& v) {

    for(int i = 0; i<4; i++)
    {
        q(i)   = v(i);
        q(i+4) = 0;
    }

};

/**
* DQ constructor using 8 scalar elements
*
* Returns a DQ object with the values of vector q equal to the values of the 8 parameters 'q0' to 'q8' passed to constructor.
* To create a DQ object using this, type: 'DQ dq_object(q0,q1,q2,q3,q4,q5,q6,q7);' where 'qn' is a double type scalar.
* \param double q0,q1,q2,q3,q4,q5,q6 and q7 are the values to be copied to the member 'q'.
*/
DQ::DQ(const double& q0,const double& q1,const double& q2,const double& q3,const double& q4,const double& q5,const double& q6,const double& q7) {

    q(0) = q0;
    q(1) = q1;
    q(2) = q2;
    q(3) = q3;
    q(4) = q4;
    q(5) = q5;
    q(6) = q6;
    q(7) = q7;

    for(int n = 0; n < 8; n++)
    {
      if(fabs(q(n)) < DQ_threshold)
        q(n) = 0;
    }

};

/**
* DQ Destructor
*
* Deletes from memory the DQ object caller. To use this destructor, type: 'dq_object.~DQ();'. Dont need parameters.
*/
DQ::~DQ(){};


// Public constant methods


/**
* Returns a constant DQ object representing the primary part of the DQ object caller.
*
* Creates a dual quaternion with values (q(0),q(1),q(2),q(3),0,0,0,0) and return. The q elements are from the DQ object caller.
* To use this member function, type: 'dq_object.P();'.
* \return A constant DQ object.
* \sa DQ(double q0,double q1,double q2,double q3).
*/
DQ DQ::P() const{
    return DQ(q(0),q(1),q(2),q(3));
};


/**
* Returns a constant DQ object representing the dual part of the DQ object caller.
*
* Creates a dual quaternion with values (q(4),q(5),q(6),q(7),0,0,0,0) and return. The q elements are from the DQ object caller.
* To use this member function, type: 'dq_object.D();'.
* \return A constant DQ object.
* \sa DQ(double q0,double q1,double q2,double q3).
*/
DQ DQ::D() const{
    return DQ(q(4),q(5),q(6),q(7));
};


/**
* Returns a constant DQ object representing the real part of the DQ object caller.
* Actually this function does the same as Re() changing only the way of calling, which is DQ::Re(dq_object).
*/
DQ DQ::Re() const{
 return DQ(q(0),0,0,0,q(4),0,0,0);
};

/**
* Returns a constant DQ object representing the imaginary part of the DQ object caller.
*
* Creates a dual quaternion with values (0,q(1),q(2),q(3),0,q(5),q(6),q(7)) and return. The q elements are from the DQ object caller.
* To use this member function, type: 'dq_object.Im();'.
* \return A constant DQ object.
* \sa DQ(double q0,double q1,double q2,double q3,double q4,double q5,double q6,double q7).
*/
DQ DQ::Im() const{
    return DQ(0,q(1),q(2),q(3),0,q(5),q(6),q(7));
};

/**
* Returns a constant DQ object representing the conjugate of the DQ object caller.
*
* Creates a dual quaternion with values (q(0),-q(1),-q(2),-q(3),q(4),-q(5),-q(6),-q(7)) and return.
* The q elements are from the DQ object caller. To use this member function, type: 'dq_object.conj();'.
* \return A constant DQ object.
* \sa DQ(double q0,double q1,double q2,double q3,double q4,double q5,double q6,double q7).
*/
DQ DQ::conj() const{
    return DQ(q(0),-q(1),-q(2),-q(3),q(4),-q(5),-q(6),-q(7));
};

/**
* Returns a constant DQ object representing the norm of the DQ object caller.
*
* Creates a dual quaternion with calculated values for the norm and return a DQ object.
* To use this member function, type: 'dq_object.norm();'.
* \return A constant DQ object.
* \sa DQ().
*/
DQ DQ::norm() const{
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
        norm.q(4) = norm.q(4)/(2*norm.q(0));

        // using threshold to verify zero values in DQ to be returned
        for(int n = 0; n < 8; n++) {
            if(fabs(norm.q(n)) < DQ_threshold )
                norm.q(n) = 0;
        }

        return norm;
    }
};


/**
* Returns a constant DQ object representing the inverse of the DQ object caller.
*
* Creates a dual quaternion with calculated values for the inverse and return a DQ object.
* To use this member function, type: 'dq_object.inv();'.
* \return A constant DQ object.
* \sa DQ(), DQ(double q0,double q1,double q2,double q3,double q4,double q5,double q6,double q7).
*/
DQ DQ::inv() const{
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
        if(fabs(inv.q(n)) < DQ_threshold )
            inv.q(n) = 0;
    }

	return inv;
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
DQ DQ::translation() const
{
  //Verify if unit quaternion
  if (this->norm() != 1)
  {
    throw(std::range_error("Bad translation() call: Not a unit dual quaternion"));
  }

  //translation part calculation
  DQ translation = this->P();
     translation = (2.0 * this->D() * translation.conj() );

  // using threshold to verify zero values in DQ to be returned
  for(int n = 0; n < 8; n++) 
    {
    if(fabs(translation.q(n)) < DQ_threshold )
      translation.q(n) = 0;
    }

  return translation;
};

/**
* Returns a constant DQ object representing the rotation axis (nx*i + ny*j + nz*k) of the unit DQ object caller.
*
* Creates a dual quaternion with calculated values for the rotation axis and return a DQ object.
* To use this member function, type: 'dq_object.rot_axis();'.
* \return A constant DQ object.
* \sa DQ().
*/
DQ DQ::rot_axis() const{

	// Verify if the object caller is a unit DQ
	if (this->norm() != 1) {
    throw(std::range_error("Bad rot_axis() call: Not a unit dual quaternion"));
  }

  double phi = acos(this->q(0));
  if(phi == 0)
    return k_; // DQ(0,0,0,1). This is only a convention;
  else
  {
    //rotation axis calculation
    DQ rot_axis = this->P();
       rot_axis = ( rot_axis.Im() * (1/sin(phi)) );

    // using threshold to verify zero values in DQ to be returned
    for(int n = 0; n < 8; n++)
    {
      if(fabs(rot_axis.q(n)) < DQ_threshold )
        rot_axis.q(n) = 0;
    }

  return rot_axis;
  }

};

/**
* Returns a constant double value representing the rotation angle in rad/s of the unit DQ object caller.
*
* Creates a double value with calculated rotation angle of a unit DQ object and return this angle in rad/s unit.
* To use this member function, type: 'dq_object.rot_angle();'.
* \return A constant double value.
* \sa DQ().
*/
double DQ::rot_angle() const{

	// Verify if the object caller is a unit DQ
	if (this->norm() != 1) {
    throw(std::range_error("Bad rot_angle() call: Not a unit dual quaternion"));
  }

  //Rotation angle calculation
  double rot_angle = 2*acos(this->q(0));

  return rot_angle;
};

/**
* Returns a constant DQ object representing the logaritm of the unit DQ object caller.
*
* Creates a dual quaternion with calculated values for the logaritm and return a DQ object.
* To use this member function, type: 'dq_object.log();'.
* \return A constant DQ object.
* \sa DQ(), DQ(double q0,double q1,double q2,double q3,double q4,double q5,double q6,double q7).
*/
DQ DQ::log() const{

	// Verify if the object caller is a unit DQ
	if (this->norm() != 1) {
    throw(std::range_error("Bad log() call: Not a unit dual quaternion"));
  }

  // log calculation
  DQ p = acos(this->q(0)) * this->rot_axis(); //primary
  DQ d = 0.5 * this->translation(); //dual
  DQ log(p.q(0),p.q(1),p.q(2),p.q(3),d.q(0),d.q(1),d.q(2),d.q(3));

  // using threshold to verify zero values in DQ to be returned
  for(int n = 0; n < 8; n++)
  {
    if(fabs(log.q(n)) < DQ_threshold )
      log.q(n) = 0;
  }

  return log;

};

/**
* Returns a constant DQ object representing the exponential of the unit DQ object caller.
*
* Creates a dual quaternion with calculated values for the exponential and return a DQ object.
* To use this member function, type: 'dq_object.exp();'.
* \return A constant DQ object.
* \sa DQ().
*/
DQ DQ::exp() const{

  double phi;
  DQ prim;
  DQ exp;

  if( this->Re() != 0.0 )
  {
    throw(std::range_error("Bad exp() call: Exponential operation is defined only for pure dual quaterions."));
  }

  prim = this->P();
  phi  = prim.q.norm();

  if(phi != 0.0)
    prim = cos(phi) + (sin(phi)/phi)*this->P();
  else
    prim = DQ(1.0);

  exp = ( prim + E_*this->D()*prim );

  // using threshold to verify zero values in DQ to be returned
  for(int n = 0; n < 8; n++) {
    if(fabs(exp.q(n)) < DQ_threshold )
      exp.q(n) = 0;
  }

  return exp;

};


/**
* Returns a constant DQ object representing the tplus operator applied to the unit DQ object caller.
*
* Creates a dual quaternion with calculated values for the tplus operation and return a DQ object.
* To use this member function, type: 'dq_object.tplus();'.
* \return A constant DQ object.
* \sa DQ().
*/
DQ DQ::tplus() const{

  DQ tplus;

	// Verify if the object caller is a unit DQ
	if (this->norm() != 1) {
    throw(std::range_error("Bad tplus() call: Not a unit dual quaternion"));
  }

  // tplus operator calculation
  tplus = this->P();
  tplus = (*this) * tplus.conj();

  // using threshold to verify zero values in DQ to be returned
  for(int n = 0; n < 8; n++)
  {
    if(fabs(tplus.q(n)) < DQ_threshold )
      tplus.q(n) = 0;
  }

  return tplus;

};

/**
* Returns a constant DQ object representing the inverse of the unit DQ object caller under decompositional multiplication.
*
* Creates a dual quaternion with calculated values for the inverse using the tplus operator and return a DQ object.
* To use this member function, type: 'dq_object.pinv();'.
* \return A constant DQ object.
* \sa DQ().
*/
DQ DQ::pinv() const{


  DQ pinv;
  DQ tinv;

	// Verify if the object caller is a unit DQ
	if (this->norm() != 1) {
    throw(std::range_error("Bad pinv() call: Not a unit dual quaternion"));
  }

  // inverse calculation under decompositional multiplication
  tinv = this->conj();
  tinv = tinv.tplus() * this->tplus();
  pinv = tinv.conj()  * this->conj();

  // using threshold to verify zero values in DQ to be returned
  for(int n = 0; n < 8; n++)
  {
    if(fabs(pinv.q(n)) < DQ_threshold )
      pinv.q(n) = 0;
  }

  return pinv;

};

/**
* Returns a constant 4x4 double boost matrix representing the Hamilton operator H+ of primary part of the DQ object caller.
*
* Creates a 4x4 Boost matrix, fill it with values based on the elements q(0) to q(4) and return the matrix H+.
* This operator is applied only for the primary part of DQ. This is the same as consider the DQ object a quaternion.
* To use this member function, type: 'dq_object.Hplus4();'.
* \return A constant boost::numeric::ublas::matrix <double> (4,4).
*/
Matrix4d DQ::Hplus4() const{
    Matrix4d op_Hplus4(4,4);
    op_Hplus4(0,0) = q(0); op_Hplus4(0,1) = -q(1); op_Hplus4(0,2) = -q(2); op_Hplus4(0,3) = -q(3);
    op_Hplus4(1,0) = q(1); op_Hplus4(1,1) =  q(0); op_Hplus4(1,2) = -q(3); op_Hplus4(1,3) =  q(2);
    op_Hplus4(2,0) = q(2); op_Hplus4(2,1) =  q(3); op_Hplus4(2,2) =  q(0); op_Hplus4(2,3) = -q(1);
    op_Hplus4(3,0) = q(3); op_Hplus4(3,1) = -q(2); op_Hplus4(3,2) =  q(1); op_Hplus4(3,3) =  q(0);
    return op_Hplus4;
};

/**
* Returns a constant 4x4 double boost matrix representing the Hamilton operator H- of primary part of the DQ object caller.
*
* Creates a 4x4 Boost matrix, fill it with values based on the elements q(0) to q(4) and return the matrix H-.
* This operator is applied only for the primary part of DQ. This is the same as consider the DQ object a quaternion.
* To use this member function, type: 'dq_object.Hminus4();'.
* \return A constant boost::numeric::ublas::matrix <double> (4,4).
*/
Matrix4d DQ::Hminus4() const{
    Matrix4d op_Hminus4(4,4);
    op_Hminus4(0,0) = q(0); op_Hminus4(0,1) = -q(1); op_Hminus4(0,2) = -q(2); op_Hminus4(0,3) = -q(3);
    op_Hminus4(1,0) = q(1); op_Hminus4(1,1) =  q(0); op_Hminus4(1,2) =  q(3); op_Hminus4(1,3) = -q(2);
    op_Hminus4(2,0) = q(2); op_Hminus4(2,1) = -q(3); op_Hminus4(2,2) =  q(0); op_Hminus4(2,3) =  q(1);
    op_Hminus4(3,0) = q(3); op_Hminus4(3,1) =  q(2); op_Hminus4(3,2) = -q(1); op_Hminus4(3,3) =  q(0);
    return op_Hminus4;
};


/**
* Returns a constant 8x8 double boost matrix representing the Hamilton operator H+ of the DQ object caller.
*
* Creates a 8x8 Boost matrix, fill it with values based on the elements q(0) to q(8) and return the matrix H+.
* To use this member function, type: 'dq_object.Hplus8();'.
* \return A constant boost::numeric::ublas::matrix <double> (8,8).
*/
Matrix<double,8,8> DQ::Hplus8() const{
    Matrix<double,8,8> op_Hplus8;
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
* Returns a constant 8x8 double boost matrix representing the Hamilton operator H- of the DQ object caller.
*
* Creates a 8x8 Boost matrix, fill it with values based on the elements q(0) to q(8) and return the matrix H-.
* To use this member function, type: 'dq_object.Hminus8();'.
* \return A constant boost::numeric::ublas::matrix <double> (8,8).
*/
Matrix<double,8,8> DQ::Hminus8() const{
    Matrix<double,8,8> op_Hminus8(8,8);
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
* Returns a constant 4x1 double Boost matrix representing the 'vec' operator of primary part of the DQ object caller.
*
* Creates a 4x1 Boost matrix, fill it with values based on the elements q(0) to q(4) and return the column matrix vec4.
* This operator is applied only for the primary part of DQ. This is the same as consider the DQ object a quaternion.
* To use this member function, type: 'dq_object.vec4();'.
* \return A constant boost::numeric::ublas::matrix <double> (4,1).
*/
Vector4d DQ::vec4() const{
    Vector4d op_vec4(4,1);
    op_vec4(0,0) = q(0);
    op_vec4(1,0) = q(1);
    op_vec4(2,0) = q(2);
    op_vec4(3,0) = q(3);
    return op_vec4;
};

/**
* Returns a constant 8x1 double boost matrix representing the 'vec' operator of the DQ object caller.
*
* Creates a 8x1 Boost matrix, fill it with values based on the elements q(0) to q(8) and return the column matrix vec8.
* To use this member function, type: 'dq_object.vec8();'.
* \return A constant boost::numeric::ublas::matrix <double> (8,1).
*/
Matrix<double,8,1>  DQ::vec8() const{
    Matrix<double,8,1>  op_vec8(8,1);
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

/** Returns the Generalized Jacobian; that it, the Jacobian that satisfies the relation Geometric_Jacobian = G * DQ_Jacobian.
* To use this member function type: 'dq_object.jacobG(x_E).
* \param DQ x_E is the dual position quaternion
* \return A constant boost::numeric::ublas::matrix <double>
*/
Matrix<double,8,8> DQ::jacobG() const{
    Matrix<double,8,8> jacobGen(8,8);
    jacobGen(0,0) = q(4); jacobGen(0,1) =  q(5); jacobGen(0,2) =  q(6); jacobGen(0,3) =  q(7);
    jacobGen(1,0) = q(5); jacobGen(1,1) = -q(4); jacobGen(1,2) =  q(7); jacobGen(1,3) = -q(6);
    jacobGen(2,0) = q(6); jacobGen(2,1) = -q(7); jacobGen(2,2) = -q(4); jacobGen(2,3) =  q(5);
    jacobGen(3,0) = q(7); jacobGen(3,1) =  q(6); jacobGen(3,2) = -q(5); jacobGen(3,3) = -q(4);

    jacobGen(0,4) =  q(0); jacobGen(0,5) =  q(1); jacobGen(0,6) =  q(2); jacobGen(0,7) =  q(3);
    jacobGen(1,4) = -q(1); jacobGen(1,5) =  q(0); jacobGen(1,6) = -q(3); jacobGen(1,7) =  q(2);
    jacobGen(2,4) = -q(2); jacobGen(2,5) =  q(3); jacobGen(2,6) =  q(0); jacobGen(2,7) = -q(1);
    jacobGen(3,4) = -q(3); jacobGen(3,5) = -q(2); jacobGen(3,6) =  q(1); jacobGen(3,7) =  q(0);

    jacobGen(4,0) =  q(0); jacobGen(4,1) =  q(1); jacobGen(4,2) =  q(2); jacobGen(4,3) =  q(3);
    jacobGen(5,0) = -q(1); jacobGen(5,1) =  q(0); jacobGen(5,2) = -q(3); jacobGen(5,3) =  q(2);
    jacobGen(6,0) = -q(2); jacobGen(6,1) =  q(3); jacobGen(6,2) =  q(0); jacobGen(6,3) = -q(1);
    jacobGen(7,0) = -q(3); jacobGen(7,1) = -q(2); jacobGen(7,2) =  q(1); jacobGen(7,3) =  q(0);

    jacobGen(4,4) = 0; jacobGen(4,5) = 0; jacobGen(4,6) = 0; jacobGen(4,7) = 0;
    jacobGen(5,4) = 0; jacobGen(5,5) = 0; jacobGen(5,6) = 0; jacobGen(5,7) = 0;
    jacobGen(6,4) = 0; jacobGen(6,5) = 0; jacobGen(6,6) = 0; jacobGen(6,7) = 0;
    jacobGen(7,4) = 0; jacobGen(7,5) = 0; jacobGen(7,6) = 0; jacobGen(7,7) = 0;

    return 2*jacobGen;
};

/**
* Generalized Jacobian, i.e, the Jacobian that satisfies the relation Geometric Jacobian = Generalized Jacobian * Analytical (DQ) Jacobian.
*
* @param x_e.
* @return A 8x8 matrix representing the Generalized Jacobian given the arguments.
*/
Matrix<double,8,8> DQ::generalizedJacobian() const
{
    return jacobG();
}


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
DQ DQ::unitDQ(const double& rot_angle, const int& x_axis,const int& y_axis,const int& z_axis, const double& x_trans,const double& y_trans, const double& z_trans) {
    if ((x_axis != 0 && x_axis != 1) || (y_axis != 0 && y_axis != 1) || (z_axis != 0 && z_axis != 1)) {
        throw(std::range_error("Bad unitDQ() call: X, Y and Z axis parameters should be 1 OR 0"));
    }
    Vector4d axis(4), translation(4);
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
    DQ h = r + E_*0.5*p*r;

    // using threshold to verify zero values in DQ to be returned
    for(int n = 0; n < 8; n++) {
        if(fabs(h.q(n)) < DQ_threshold )
            h.q(n) = 0;
    }
    return h;
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
            if(fabs(dq.q(n)) < DQ_threshold )
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
            if(fabs(dq.q(n)) < DQ_threshold )
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
            if(fabs(dq.q(n)) < DQ_threshold )
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
        if(fabs(q(n) - dq2.q(n)) > DQ_threshold )
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
        if(fabs(q(n) - dq2.q(n)) > DQ_threshold )
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

    if (dq1.norm() != 1) {
        throw(std::range_error("Bad operator^() call: Not a unit dual quaternion"));
    }
    else {
        DQ dq;
        dq = m * dq1.log();
        dq = dq.exp();
        for(int n = 0; n < 8; n++) {
                if(fabs(dq.q(n)) < DQ_threshold )
                    dq.q(n) = 0;
            }
        return dq;
    }
};


std::ostream& operator<<(std::ostream& os, DQ dq)
{
    os << dq.q(0) << " "
       << dq.q(1) << "i "
       << dq.q(2) << "j "
       << dq.q(3) << "k +E( "
       << dq.q(4) << " "
       << dq.q(5) << "i "
       << dq.q(6) << "j "
       << dq.q(7) << "k )";

    return os;
};


}//Namespace DQRobotics
