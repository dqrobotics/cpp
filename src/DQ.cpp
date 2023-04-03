/**
(C) Copyright 2011-2023 DQ Robotics Developers

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
1. Bruno Vilhena Adorno (adorno@ieee.org)
        - Responsible for the original implementation.
          [bvadorno committed on Jul 20, 2012](7368f3e)
          (https://github.com/dqrobotics/cpp/commit/7368f3ea3d557834661d723adde981250db0b87f).

2. Mateus Rodrigues Martins (martinsrmateus@gmail.com)
        - Added new methods, and support for Boost library.
          [mateusmartins committed on Jul 27, 2012]()
          (https://github.com/dqrobotics/cpp/commit/7d96efb354ffa07a093d5cb3f34af2c7ce8e2d39).

3. Murilo M. Marinho (murilo@nml.t.u-tokyo.ac.jp)
       - Refactoring, and compliance with the new style. 
         [murilomarinho committed on Dec 22, 2012](c7f4596)
         (https://github.com/dqrobotics/cpp/commit/c7f459612bb47ab2151b64ed6820c9f6fb242fa6).

       - Added support for Eigen library
         [murilomarinho committed on Jan 31, 2013](1ec0bf0)
         (https://github.com/dqrobotics/cpp/commit/1ec0bf096ff7b9f3f73ee0513f0a6f07c2a58f01).

4. Marcos da Silva Pereira (marcos.si.pereira@gmail.com)
        - Translated the Q4 and the Q8 methods from the MATLAB implementation in PR #56 
          (https://github.com/dqrobotics/cpp/pull/56).
*/

#include <dqrobotics/DQ.h>
#include <sstream>
#include <stdexcept> //for range_error
#include <vector>
#include <cmath>

namespace DQ_robotics{

//To comply with MATLAB
const DQ DQ::i(0,1,0,0,0,0,0,0);
const DQ DQ::j(0,0,1,0,0,0,0,0);
const DQ DQ::k(0,0,0,1,0,0,0,0);
const DQ DQ::E(0,0,0,0,1,0,0,0);

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
 * @brief rotation Obtain the rotation of a unit DQ.
 * @param dq the input DQ.
 * @return the rotation of \p dq.
 * @exception Will throw a std::range_error iff the input is a not unit DQ.
 */
DQ rotation(const DQ& dq)
{
    return dq.rotation();
}

/**
* Rotation Axis operator -> retrieves the Rotation Axis represented by a DQ.
*
* @param dq The DQ which Rotation Axis you wish.
* @return a constant DQ representing the Rotation Axis represented by dq.
*/
DQ rotation_axis(const DQ& dq)
{
    return dq.rotation_axis();
}

/**
* Rotation Angle operator -> retrieves the Rotation Angle represented by a DQ.
*
* @param dq The DQ which Rotation Angle you wish.
* @return a constant DQ representing the Rotation Angle represented by dq.
*/
double rotation_angle(const DQ& dq)
{
    return dq.rotation_angle();
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

DQ pow(const DQ& dq, const double& a)
{
    return dq.pow(a);
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
Matrix4d hamiplus4(const DQ& dq)
{
    return dq.hamiplus4();
}

/**
* Hamilton operator H- for the primary part.
*
* @param dq The DQ which H- you wish.
* @return the 4x4 matrix representing the H- operator of the primary part of dq.
*/
Matrix4d haminus4(const DQ& dq)
{
    return dq.haminus4();
}

/**
* Hamilton operator H+.
*
* @param dq The DQ which H+ you wish.
* @return the 8x8 matrix representing the H+ operator of dq.
*/
Matrix<double,8,8> hamiplus8(const DQ& dq)
{
    return dq.hamiplus8();
}

/**
* Hamilton operator H-.
*
* @param dq The DQ which H- you wish.
* @return the 8x8 matrix representing the H- operator of dq.
*/
Matrix<double,8,8> haminus8(const DQ& dq)
{
    return dq.haminus8();
}

Vector3d vec3(const DQ& dq)
{
    return dq.vec3();
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


Matrix<double,6,1> vec6(const DQ& dq)
{
    return dq.vec6();
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

Matrix4d crossmatrix4(const DQ& dq)
{
    Matrix4d cm;
    cm << 0, 0, 0, 0,
            0, 0, -dq.q(3), dq.q(2),
            0, dq.q(3), 0, -dq.q(1),
            0, -dq.q(2), dq.q(1), 0;
    return cm;
}

/**
* Generalized Jacobian, i.e, the Jacobian that satisfies the relation Geometric Jacobian = Generalized Jacobian * Analytical (DQ) Jacobian.
*
* @param param_dq.
* @param x_e.
* @return A 8x8 matrix representing the Generalized Jacobian given the arguments.
*/
Matrix<double,8,8> generalized_jacobian(const DQ& x_E)
{
    return x_E.generalized_jacobian();
}

/**
* Dual quaternion normalization
* @param dq
* @return the normalized version of qd.
*/
DQ normalize(const DQ& dq)
{
    return dq*(dq.norm().inv());
}

DQ sharp(const DQ& dq)
{
    return dq.sharp();
}

/**
 * @brief The dot product between \p a and \p b
 * @param a the left side DQ.
 * @param b the right side DQ.
 * @return the dot product between \p a and \p b
 * @exception Will throw a std::range_error iff one of the inputs is not imaginary.
 */
DQ dot(const DQ& dq1, const DQ& dq2)
{
    /*if(dq1!=Im(dq1) || dq2!=Im(dq2))
    {
        throw std::range_error("One of the inputs is not imaginary in dot");
    }*/
    return -1.0*(dq1*dq2+dq2*dq1)*0.5;
}

/**
 * @brief The cross product between \p a and \p b
 * @param a the left side DQ.
 * @param b the right side DQ.
 * @return the cross product between \p a and \p b
 * @exception Will throw a std::range_error iff one of the inputs is not imaginary.
 */
DQ cross(const DQ& dq1, const DQ& dq2)
{
    /*if(dq1!=Im(dq1) || dq2!=Im(dq2))
    {
        throw std::range_error("One of the inputs is not imaginary in cross");
    }*/
    return (dq1*dq2-dq2*dq1)*0.5;
}

DQ Ad(const DQ& dq1, const DQ& dq2)
{
    return (dq1)*(dq2)*conj(dq1);
}

DQ Adsharp(const DQ& dq1, const DQ& dq2)
{
    return sharp(dq1)*dq2*conj(dq1);
}

Matrix<double,4,3> Q4(const DQ& dq) 
{    
    return dq.Q4();
}

Matrix<double,8,6> Q8(const DQ& dq) 
{    
    return dq.Q8();
}

/****************************************************************
**************DQ CLASS METHODS***********************************
*****************************************************************/

VectorXd DQ::q_() const
{
    return q;
}

double DQ::q_(const int a) const
{
    return q(a);
}

DQ::DQ(VectorXd &&v)
{
    switch (v.size())
    {
    //An eight-dimensional v contains the coefficients of general dual quaternions
    case 8:
        q = std::move(v);
        break;
        //A six-dimensional v contains the coefficientes of a pure dual quaternion
    case 6:
        q << 0.0 , v(0), v(1), v(2),
                0.0 , v(3), v(4), v(5);
        break;
        //A four-dimensional v contains the coefficients of a general quaternion
    case 4:
        q << v(0), v(1), v(2), v(3),
                0.0 , 0.0 , 0.0 , 0.0 ;
        break;
        //A three-dimensional v contains the coefficients of a pure quaternion
    case 3:
        q << 0.0 , v(0), v(1), v(2),
                0.0 , 0.0 , 0.0 , 0.0 ;
        break;
        //An one-dimensional v contains the coefficient of a real number.
    case 1:
        q << v(0), 0.0 , 0.0 , 0.0,
                0.0 , 0.0 , 0.0 , 0.0 ;
        break;
    default:
        throw std::range_error(std::string("Trying to initialize a DQ with a size of ") + std::to_string(v.size()) + std::string(" which is not allowed."));
    }
}


/**
 * @brief DQ::DQ Constructor using a VectorXd.
 * @param v a VectorXd with size 1, 3, 4, 6, or 8. It will be read treated as a real number,
 * a pure quaternion, a general quaternion, a pure dual quaternion, or a general dual quaternion, respectively.
 * @exception std::range_error if the size is different from the ones specified above.
 */
DQ::DQ(const VectorXd& v)
{
    switch (v.size())
    {
    //An eight-dimensional v contains the coefficients of general dual quaternions
    case 8:
        q << v(0), v(1), v(2), v(3),
                v(4), v(5), v(6), v(7);
        break;
        //A six-dimensional v contains the coefficientes of a pure dual quaternion
    case 6:
        q << 0.0 , v(0), v(1), v(2),
                0.0 , v(3), v(4), v(5);
        break;
        //A four-dimensional v contains the coefficients of a general quaternion
    case 4:
        q << v(0), v(1), v(2), v(3),
                0.0 , 0.0 , 0.0 , 0.0 ;
        break;
        //A three-dimensional v contains the coefficients of a pure quaternion
    case 3:
        q << 0.0 , v(0), v(1), v(2),
                0.0 , 0.0 , 0.0 , 0.0 ;
        break;
        //An one-dimensional v contains the coefficient of a real number.
    case 1:
        q << v(0), 0.0 , 0.0 , 0.0,
                0.0 , 0.0 , 0.0 , 0.0 ;
        break;
    default:
        throw std::range_error(std::string("Trying to initialize a DQ with a size of ") + std::to_string(v.size()) + std::string(" which is not allowed."));
    }
}

/**
 * @brief DQ::DQ Constructor using up to 8 double values. The constructor will return a DQ q equivalent to
 * q = q0 + q1*i_ + q2*j_ + q3*k_ + E_*(q4 + q5*i_ + q6*j_ + q7*k_).
 * @param q0 a double (default = 0.0)
 * @param q1 a double (default = 0.0)
 * @param q2 a double (default = 0.0)
 * @param q3 a double (default = 0.0)
 * @param q4 a double (default = 0.0)
 * @param q5 a double (default = 0.0)
 * @param q6 a double (default = 0.0)
 * @param q7 a double (default = 0.0)
 */
DQ::DQ(const double& q0,const double& q1,const double& q2,const double& q3,const double& q4,const double& q5,const double& q6,const double& q7) noexcept:
    q((Matrix<double,8,1>() << q0,q1,q2,q3,q4,q5,q6,q7).finished())
{

}



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
}


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
}


/**
* Returns a constant DQ object representing the real part of the DQ object caller.
* Actually this function does the same as Re() changing only the way of calling, which is DQ::Re(dq_object).
*/
DQ DQ::Re() const{
    return DQ(q(0),0,0,0,q(4),0,0,0);
}

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
}

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
}

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
}


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

    return inv;
}


/**
* Returns a constant DQ object representing the translation part of the unit DQ object caller.
*
* Creates a dual quaternion with calculated values for the translation part and return a DQ object.
* Assuming  dq=r + DQ.E * p * r * (0.5), the return is p.
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

    return translation;
}

/**
 * @brief rotation Obtain the rotation of a unit DQ.
 * @return the rotation of this DQ.
 * @exception Will throw a std::range_error iff the input is a not unit DQ.
 */
DQ DQ::rotation() const
{
    //Verify if unit quaternion
    if (this->norm() != 1)
    {
        throw(std::range_error("Bad rotation() call: Not a unit dual quaternion"));
    }
    const DQ rotation = this->P();

    return rotation;
}

/**
* Returns a constant DQ object representing the rotation axis (nx*i + ny*j + nz*k) of the unit DQ object caller.
*
* Creates a dual quaternion with calculated values for the rotation axis and return a DQ object.
* To use this member function, type: 'dq_object.rot_axis();'.
* \return A constant DQ object.
* \sa DQ().
*/
DQ DQ::rotation_axis() const{

    // Verify if the object caller is a unit DQ
    if (this->norm() != 1) {
        throw(std::range_error("Bad rot_axis() call: Not a unit dual quaternion"));
    }

    double phi = rotation_angle()/2.0;
    if(phi == 0.0)
        return k_; // DQ(0,0,0,1). This is only a convention;
    else
    {
        //rotation axis calculation
        DQ rot_axis = this->P();
        rot_axis = ( rot_axis.Im() * (1/sin(phi)) );

        return rot_axis;
    }

}

/**
* Returns a constant double value representing the rotation angle in rad/s of the unit DQ object caller.
*
* Creates a double value with calculated rotation angle of a unit DQ object and return this angle in rad/s unit.
* To use this member function, type: 'dq_object.rot_angle();'.
* \return A constant double value.
* \sa DQ().
*/
double DQ::rotation_angle() const{

    // Verify if the object caller is a unit DQ
    if (this->norm() != 1) {
        throw(std::range_error("Bad rot_angle() call: Not a unit dual quaternion"));
    }

    //Hotfix for issue #25, we might need to think more about this one in the future
    double rot_angle;
    if(this->q(0) > 1.0)
        rot_angle = 2*acos(1.0);
    else if(this->q(0) < -1.0)
        rot_angle = 2*acos(-1.0);
    else
        rot_angle = 2*acos(this->q(0));

    return rot_angle;
}

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
    DQ p = (0.5 * this->rotation_angle()) * this->rotation_axis(); //primary
    DQ d = 0.5 * this->translation(); //dual
    DQ log(p.q(0),p.q(1),p.q(2),p.q(3),d.q(0),d.q(1),d.q(2),d.q(3));

    return log;

}

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

    return exp;

}

/**

*/
DQ DQ::pow(const double a) const
{
    DQ logtimesa = a*this->log();
    return logtimesa.exp();
}

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

    return tplus;

}

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

    return pinv;

}

/**
* Returns a constant 4x4 double boost matrix representing the Hamilton operator H+ of primary part of the DQ object caller.
*
* Creates a 4x4 Boost matrix, fill it with values based on the elements q(0) to q(4) and return the matrix H+.
* This operator is applied only for the primary part of DQ. This is the same as consider the DQ object a quaternion.
* To use this member function, type: 'dq_object.Hplus4();'.
* \return A constant boost::numeric::ublas::matrix <double> (4,4).
*/
Matrix4d DQ::hamiplus4() const{
    Matrix4d op_Hplus4(4,4);
    op_Hplus4(0,0) = q(0); op_Hplus4(0,1) = -q(1); op_Hplus4(0,2) = -q(2); op_Hplus4(0,3) = -q(3);
    op_Hplus4(1,0) = q(1); op_Hplus4(1,1) =  q(0); op_Hplus4(1,2) = -q(3); op_Hplus4(1,3) =  q(2);
    op_Hplus4(2,0) = q(2); op_Hplus4(2,1) =  q(3); op_Hplus4(2,2) =  q(0); op_Hplus4(2,3) = -q(1);
    op_Hplus4(3,0) = q(3); op_Hplus4(3,1) = -q(2); op_Hplus4(3,2) =  q(1); op_Hplus4(3,3) =  q(0);
    return op_Hplus4;
}

/**
* Returns a constant 4x4 double boost matrix representing the Hamilton operator H- of primary part of the DQ object caller.
*
* Creates a 4x4 Boost matrix, fill it with values based on the elements q(0) to q(4) and return the matrix H-.
* This operator is applied only for the primary part of DQ. This is the same as consider the DQ object a quaternion.
* To use this member function, type: 'dq_object.Hminus4();'.
* \return A constant boost::numeric::ublas::matrix <double> (4,4).
*/
Matrix4d DQ::haminus4() const{
    Matrix4d op_Hminus4(4,4);
    op_Hminus4(0,0) = q(0); op_Hminus4(0,1) = -q(1); op_Hminus4(0,2) = -q(2); op_Hminus4(0,3) = -q(3);
    op_Hminus4(1,0) = q(1); op_Hminus4(1,1) =  q(0); op_Hminus4(1,2) =  q(3); op_Hminus4(1,3) = -q(2);
    op_Hminus4(2,0) = q(2); op_Hminus4(2,1) = -q(3); op_Hminus4(2,2) =  q(0); op_Hminus4(2,3) =  q(1);
    op_Hminus4(3,0) = q(3); op_Hminus4(3,1) =  q(2); op_Hminus4(3,2) = -q(1); op_Hminus4(3,3) =  q(0);
    return op_Hminus4;
}


/**
* Returns a constant 8x8 double boost matrix representing the Hamilton operator H+ of the DQ object caller.
*
* Creates a 8x8 Boost matrix, fill it with values based on the elements q(0) to q(8) and return the matrix H+.
* To use this member function, type: 'dq_object.Hplus8();'.
* \return A constant boost::numeric::ublas::matrix <double> (8,8).
*/
Matrix<double,8,8> DQ::hamiplus8() const{
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
}

/**
* Returns a constant 8x8 double boost matrix representing the Hamilton operator H- of the DQ object caller.
*
* Creates a 8x8 Boost matrix, fill it with values based on the elements q(0) to q(8) and return the matrix H-.
* To use this member function, type: 'dq_object.Hminus8();'.
* \return A constant boost::numeric::ublas::matrix <double> (8,8).
*/
Matrix<double,8,8> DQ::haminus8() const{
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
}

Vector3d DQ::vec3() const
{
    Vector3d ret;
    ret << q(1),q(2),q(3);
    return ret;
}

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
}


Matrix<double,6,1> DQ::vec6() const
{
    Matrix<double,6,1> ret;
    ret << q(1),q(2),q(3),q(5),q(6),q(7);
    return ret;
}

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
}

/** Returns the Generalized Jacobian; that it, the Jacobian that satisfies the relation Geometric_Jacobian = G * DQ_Jacobian.
* To use this member function type: 'dq_object.jacobG(x_E).
* \param DQ x_E is the dual position quaternion
* \return A constant boost::numeric::ublas::matrix <double>
*/
Matrix<double,8,8> DQ::generalized_jacobian() const{
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
}


DQ DQ::normalize() const
{
    return (*this)*((*this).norm().inv());
}


DQ DQ::sharp() const
{
    return (P()-E_*D());
}

DQ DQ::Ad(const DQ &dq2) const
{
    return DQ_robotics::Ad(*this,dq2);
}

DQ DQ::Adsharp(const DQ& dq2) const
{
    return DQ_robotics::Adsharp(*this,dq2);
}

/** 
* @brief Given the unit quaternion r, return the partial derivative of vec4(r) with respect to vec3(log(r)).
*        Eq. (22) of Savino et al (2020). Pose consensus based on dual quaternion algebra with application 
*        to decentralized formation control of mobile manipulators.
*        https://doi.org/10.1016/j.jfranklin.2019.09.045
* @returns A matrix representing the desired partial derivative.
*/
Matrix<double,4,3> DQ::Q4() const
{
    if (!is_unit(*this) || !is_quaternion(*this))
    {
        throw(std::range_error("Bad Q4() call: Not a unit quaternion"));
    }

    const Vector4d r = this->vec4();
    const double phi = double(this->rotation_angle());
    const Vector3d n = this->rotation_axis().vec3();
    const double nx = n(0);
    const double ny = n(1);
    const double nz = n(2);

    double theta;
    if (phi == 0)
    {
        theta = 1;
    }
    else
    {
        theta = sin(phi/2.0)/(phi/2.0);
    }
    
    double gamma = r(0) - theta;

    Matrix<double,4,3> Q4(4,3);
    Q4 << -r(1),                        -r(2),                       -r(3),
          gamma*std::pow(nx,2)+theta,   gamma*nx*ny,                 gamma*nx*nz,
          gamma*nx*ny,                  gamma*std::pow(ny,2)+theta,  gamma*ny*nz,
          gamma*nz*nx,                  gamma*nz*ny,                 gamma*std::pow(nz,2)+theta;

    return Q4;
}

/** 
* @brief Given the unit dual quaternion x, Q8(x) returns the partial derivative of vec8(x) with respect to vec6(log(x)).
*        Theorem 4 of Savino et al (2020). Pose consensus based on dual quaternion algebra with application 
*        to decentralized formation control of mobile manipulators.
*        https://doi.org/10.1016/j.jfranklin.2019.09.045
* @returns A matrix representing the desired partial derivative.
*/
Matrix<double,8,6> DQ::Q8() const
{
    if (!is_unit(*this))
    {
        throw(std::range_error("Bad Q8() call: Not a unit dual quaternion"));
    }

    const DQ r = this->rotation();
    const DQ p = this->translation();

    const MatrixXd Q = r.Q4();
    MatrixXd Qp(4,3);
    Qp << MatrixXd::Zero(1,3),
          MatrixXd::Identity(3,3);

    Matrix<double,8,6> Q8(8,6);

    Q8 << Q, MatrixXd::Zero(4,3),
          0.5*p.hamiplus4()*Q, r.haminus4()*Qp;

    return Q8;
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

    return h;
}

std::string DQ::to_string() const
{
    std::stringstream ss;

    std::vector<std::string> s{"",""}; // intermediate strings containing the primary and dual parts
    std::vector<std::string> aux{" ", "i", "j", "k"};
    std::vector<bool> has_element{false,false};
    int shift = 4;

    //iterate over the primary part and dual part of the dual quaternion
    for (int j = 0; j < 2; j++)
    {
        // when j = 0, shift = 0 and iterate over the primary part
        // when j = 1, shift = 4 and iterate over the dual part
        shift = 4*j;

        //iterate over the quaternion coefficients
        for (int i = 0; i < 4; i++)
        {
            if (fabs(q(i+shift)) > DQ_threshold) // To avoid displaying values very close to zero
            {
                std::string quat_coefficient = std::to_string(fabs(q(i+shift)));

                // retrieves the index pointing to the last character that is not zero.
                // it can be a nonzero numeral or a decimal point.
                int index_of_nonzero = quat_coefficient.find_last_not_of('0');

                // if it is a decimal dot, we want to remove it.
                // if quat_coefficient[index_of_nonzero] == ".", then the compare method returns 0
                if (quat_coefficient.compare(index_of_nonzero, 1, std::string{"."}) != 0)
                {
                    // the last nonzero element is a nonzero numeral, therefore we do not want to remove it.
                    index_of_nonzero++;
                }

                // erase all unnecessary trailing zeros
                // if the decimal point is not necessary either, also erase it
                quat_coefficient.erase (index_of_nonzero, std::string::npos );

                // if it is a negative number, display the negative sign
                if (q(i+shift) < 0)
                {
                    s[j] = s[j] + std::string(" - ");
                }
                // if it is the first element AND a positive number, then has_element must be false
                // and there is no need to print '+'
                else if (has_element[j] == true)
                {
                    s[j] = s[j] + std::string(" + ");
                }

                //if it's the first element, regardless of its sign
                if (i == 0)
                {
                    s[j] = s[j] + quat_coefficient;
                }
                //if it's not the first element, then we have to print one imaginary unit
                else
                {
                    s[j] = s[j] + quat_coefficient + aux[i];
                }
                has_element[j] = true;
            }
        }
    }

    // the dual quaternion is not zero
    if (has_element[0] == true || has_element[1] == true)
    {
        // if the dual part is not zero, then we have to show the multiplication by the dual unit
        if (has_element[1] == true)
        {
            s[1] = "E*(" + s[1] + ")";

            //also, if the primary part is not zero, then we have to show the addition sign before the dual part
            if (has_element[0] == true)
            {
                s[1] = " + " + s[1];
            }
        }
        ss << s[0] + s[1];
    }
    else
    {
        ss << "0";
    }

    return ss.str();
}

DQ::operator double() const
{
    return q(0);
}

DQ::operator int() const
{
    return static_cast<int>(q(0));
}


//Overloaded operators definitions

/**
 * @brief operator + between two DQs.
 * @param dq1 the first DQ.
 * @param dq2 the second DQ.
 * @return the (dual quaternion) sum between two DQs, that is ret = dq1 + dq2.
 */
const DQ operator +(const DQ& dq1, const DQ& dq2) noexcept
{
    return DQ(dq1.q + dq2.q);
}

/**
 * @brief operator + between two DQs when the first argument
 * is an rvalue.
 * @param rdq1 an rvalue DQ.
 * @param dq2 an lvalue DQ.
 * @return the (dual quaternion) sum  between two DQs.
 */
const DQ operator +(DQ&& rdq1, const DQ& dq2) noexcept
{
    rdq1.q+=dq2.q;
    return std::move(rdq1);
}

/**
 * @brief operator + between two DQs when the first argument
 * is an rvalue.
 * @param dq1 an lvalue DQ.
 * @param rdq2 an rvalue DQ.
 * @return the (dual quaternion) sum  between two DQs.
 */
const DQ operator +(const DQ& dq1, DQ&& rdq2) noexcept
{
    rdq2.q+=dq1.q;
    return std::move(rdq2);
}

/**
 * @brief operator + between two DQs when both arguments
 * are rvalues
 * @param rdq1 an rvalue DQ.
 * @param rdq2 an rvalue DQ.
 * @return the (dual quaternion) sum between two DQs.
*/
const DQ operator +(DQ&& rdq1, DQ&& rdq2) noexcept
{
    rdq1.q+=rdq2.q;
    return std::move(rdq1);
}

/**
 * @brief operator - between two DQs.
 * @param dq1 the first DQ.
 * @param dq2 the second DQ.
 * @return the (dual quaternion) difference between two DQs, that is ret = dq1 - dq2.
 */
const DQ operator -(const DQ& dq1, const DQ& dq2) noexcept
{
    return DQ(dq1.q - dq2.q);
}

/**
 * @brief operator - between two DQs when the first argument
 * is an rvalue.
 * @param rdq1 an rvalue DQ.
 * @param dq2 an lvalue DQ.
 * @return the (dual quaternion) difference between two DQs.
 */
const DQ operator -(DQ&& rdq1, const DQ& dq2) noexcept
{
    rdq1.q-=dq2.q;
    return std::move(rdq1);
}


/**
 * @brief operator - between two DQs when both arguments
 * are rvalues
 * @param rdq1 an rvalue DQ.
 * @param rdq2 an rvalue DQ.
 * @return the (dual quaternion) difference between two DQs.
*/
const DQ operator -(DQ&& rdq1, DQ&& rdq2) noexcept
{
    rdq1.q-=rdq2.q;
    return std::move(rdq1);
}


DQ DQ::operator-() const
{
    return DQ(-1.0*this->q);
}

/**
 * @brief operator * between two DQs.
 * @param dq1 the first DQ.
 * @param dq2 the second DQ.
 * @return the (dual quaternion) product between two DQs, that is ret = dq1*dq2.
 */
const DQ operator *(const DQ& dq1, const DQ& dq2) noexcept
{
    //2022/07/20
    //Murilo:
    //- Removed one extra unnecessary call to DQ()
    //- Optimized this way (with the constructor call on the return) to use copy elision
    //https://en.cppreference.com/w/cpp/language/copy_elision
    //- These temporaries dq1_d, dq2_p, dq2_d currently cause three DQ constructors
    //to be called, even though they are not strictly necessary in a programming
    //point-of-view.

    const DQ& dq1_d = dq1.D();
    const DQ& dq2_p = dq2.P();
    const DQ& qd2_d = dq2.D();

    const double& q4 = dq1.q(0)*qd2_d.q(0) - dq1.q(1)*qd2_d.q(1) - dq1.q(2)*qd2_d.q(2) - dq1.q(3)*qd2_d.q(3);
    const double& q5 = dq1.q(0)*qd2_d.q(1) + dq1.q(1)*qd2_d.q(0) + dq1.q(2)*qd2_d.q(3) - dq1.q(3)*qd2_d.q(2);
    const double& q6 = dq1.q(0)*qd2_d.q(2) - dq1.q(1)*qd2_d.q(3) + dq1.q(2)*qd2_d.q(0) + dq1.q(3)*qd2_d.q(1);
    const double& q7 = dq1.q(0)*qd2_d.q(3) + dq1.q(1)*qd2_d.q(2) - dq1.q(2)*qd2_d.q(1) + dq1.q(3)*qd2_d.q(0);

    return DQ(
        dq1.q(0)*dq2.q(0) - dq1.q(1)*dq2.q(1) - dq1.q(2)*dq2.q(2) - dq1.q(3)*dq2.q(3),
        dq1.q(0)*dq2.q(1) + dq1.q(1)*dq2.q(0) + dq1.q(2)*dq2.q(3) - dq1.q(3)*dq2.q(2),
        dq1.q(0)*dq2.q(2) - dq1.q(1)*dq2.q(3) + dq1.q(2)*dq2.q(0) + dq1.q(3)*dq2.q(1),
        dq1.q(0)*dq2.q(3) + dq1.q(1)*dq2.q(2) - dq1.q(2)*dq2.q(1) + dq1.q(3)*dq2.q(0),
        q4 + dq1_d.q(0)*dq2_p.q(0) - dq1_d.q(1)*dq2_p.q(1) - dq1_d.q(2)*dq2_p.q(2) - dq1_d.q(3)*dq2_p.q(3),
        q5 + dq1_d.q(0)*dq2_p.q(1) + dq1_d.q(1)*dq2_p.q(0) + dq1_d.q(2)*dq2_p.q(3) - dq1_d.q(3)*dq2_p.q(2),
        q6 + dq1_d.q(0)*dq2_p.q(2) - dq1_d.q(1)*dq2_p.q(3) + dq1_d.q(2)*dq2_p.q(0) + dq1_d.q(3)*dq2_p.q(1),
        q7 + dq1_d.q(0)*dq2_p.q(3) + dq1_d.q(1)*dq2_p.q(2) - dq1_d.q(2)*dq2_p.q(1) + dq1_d.q(3)*dq2_p.q(0)
    );
}

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
bool DQ::operator==(const DQ& dq2) const{
    for(int n = 0; n<8; n++)
    {
        if(fabs(q(n) - dq2.q_(n)) > DQ_threshold )
            return false; //elements of Dual Quaternion different of scalar
    }
    return true; //elements of Dual Quaternion equal to scalar
}

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
bool DQ::operator!=(const DQ& dq2) const{
    for(int n = 0; n<8; n++)
    {
        if(fabs(q(n) - dq2.q(n)) > DQ_threshold )
            return true; //elements of Dual Quaternion different of scalar
    }
    return false; //elements of Dual Quaternion equal to scalar
}

std::ostream& operator<<(std::ostream& os, const DQ& dq)
{
    return os << dq.to_string();
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
 * @brief is_unit Checks if the input @p is unit norm.
 * @param dq the input DQ.
 * @return true if the norm is 1, false otherwise.
 */
bool is_unit(const DQ& dq)
{
    if(norm(dq)==1.0)
        return true;
    else
        return false;
}

/**
 * @brief is_pure Checks if the input @p dq is pure.
 * @param dq the input DQ
 * @return true if the real part of @p dq is 0, false otherwise
 */
bool is_pure(const DQ& dq)
{
    if(Re(dq)==0.0)
        return true;
    else
        return false;
}

/**
 * @brief is_real Checks if the input @p dq is real.
 * @param dq the input DQ
 * @return true if the imaginary part of @p dq is zero, false otherwise
 */
bool is_real(const DQ& dq)
{
    if(Im(dq)==0.0)
        return true;
    else
        return false;
}

/**
 * @brief is_real_number Checks if the input @p is a real number.
 * @param dq the input DQ
 * @return true if the imaginary and dual parts of @p dq are 0, false otherwise.
 */
bool is_real_number(const DQ& dq)
{
    if(Im(dq)==0.0 && D(dq)==0.0)
        return true;
    else
        return false;
}

/**
 * @brief is_quaternion Checks if the input @p is a quaternion.
 * @param dq the input DQ
 * @return true if the dual part of @p is zero, false otherwise.
 */
bool is_quaternion(const DQ& dq)
{
    if(D(dq)==0.0)
        return true;
    else
        return false;
}

/**
 * @brief is_pure_quaternion Checks if the input @p is a pure quaternion.
 * @param dq the input DQ
 * @return true if the input is both a quaternion and pure, false otherwise.
 */
bool is_pure_quaternion(const DQ& dq)
{
    if(is_pure(dq)&&is_quaternion(dq))
        return true;
    else
        return false;
}

/**
 * @brief is_line Checks if the input @p is a Plucker line.
 * @param dq the input DQ
 * @return true if the input is both pure and unit, false otherwise.
 */
bool is_line(const DQ& dq)
{
    if(is_unit(dq) && is_pure(dq))
        return true;
    else
        return false;
}

/**
 * @brief is_plane Checks if the input @p is a plane.
 * @param dq the input DQ.
 * @return true if @p is unit and if its dual part is real, false otherwise.
 */
bool is_plane(const DQ& dq)
{
    if(is_unit(dq) && is_real(D(dq)))
        return true;
    else
        return false;
}


}//Namespace DQRobotics
