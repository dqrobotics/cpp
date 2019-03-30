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
- Murilo M. Marinho        (murilo@nml.t.u-tokyo.ac.jp)
- Mateus Rodrigues Martins (martinsrmateus@gmail.com)
*/

#ifndef DQ_H
#define DQ_H

#if defined(__GNUC__) || defined(__clang__)
#define DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED __declspec(deprecated)
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED
#endif

#include <eigen3/Eigen/Dense>
#include <iostream>
using namespace Eigen;

namespace DQ_robotics{


class DQ{

    //Private methods
private:
    VectorXd q_() const;
    double q_(const int a) const;

    //Attributes
public:
    VectorXd q;

    //Static Methods
public:

    static DQ unitDQ( const double& rot_angle, const int& x_axis, const int& y_axis, const int& z_axis, const double& x_trans, const double& y_trans, const double& z_trans);
    //To comply with MATLAB
    const static DQ i;
    const static DQ j;
    const static DQ k;
    const static DQ E;

    //Methods
public:

    DQ(const VectorXd& v);

    DQ(const double& q0=0.0, const double& q1=0.0, const double& q2=0.0, const double& q3=0.0, const double& q4=0.0, const double& q5=0.0, const double& q6=0.0, const double& q7=0.0);

    ~DQ();

    //Methods
    DQ P() const;

    DQ D() const;

    DQ Re() const;

    DQ Im() const;

    DQ conj() const;

    DQ norm() const;

    DQ inv() const;

    DQ translation() const;

    DQ rotation() const;

    DQ rotation_axis() const;

    double rotation_angle() const;

    DQ log() const;

    DQ exp() const;

    DQ pow(const double a) const;

    DQ tplus() const;
    inline DQ T() const{return tplus();}

    DQ pinv() const;

    Matrix4d hamiplus4() const;

    Matrix4d haminus4() const;

    Matrix<double,8,8> hamiplus8() const;

    Matrix<double,8,8> haminus8() const;

    Vector4d vec4() const;

    Matrix<double,8,1> vec8() const;

    Matrix<double,8,8> generalized_jacobian() const;

    DQ normalize() const;

    DQ sharp() const;

    //Operator (-) Overload
    DQ operator-();
    //Operator (==) Overload
    bool operator==(const DQ& dq2) const;
    //Operator (!=) Overload
    bool operator!=(const DQ& dq2) const;

    std::string to_string() const;

    ///Deprecated signatures
    DEPRECATED Matrix<double,8,8> Hminus8() const{return haminus8();}
    DEPRECATED Matrix<double,8,8> Hplus8() const{return hamiplus8();}
    DEPRECATED Matrix4d Hminus4() const {return haminus4();}
    DEPRECATED Matrix4d Hplus4() const {return hamiplus4();}
    DEPRECATED Matrix<double,8,8> generalizedJacobian() const {return generalized_jacobian();}
    DEPRECATED Matrix<double,8,8> jacobG() const {return generalized_jacobian();}
    DEPRECATED DQ rot_axis() const {return rotation_axis();}
    DEPRECATED double rot_angle() const {return rotation_angle();}

};//DQ Class END

/*************************************************************************
 ************** DUAL QUATERNION CONSTANTS AND OPERATORS ******************
 ************************************************************************/

//Operator (+) Overload
DQ operator+(const DQ& dq1, const DQ& dq2);
DQ operator+(const DQ& dq, const int& scalar);
DQ operator+(const int& scalar, const DQ& dq);
DQ operator+(const DQ& dq, const float &scalar);
DQ operator+(const float &scalar, const DQ& dq);
DQ operator+(const DQ& dq, const double& scalar);
DQ operator+(const double& scalar, const DQ& dq);

//Operator (-) Overload
DQ operator-(const DQ& dq1, const DQ& dq2);
DQ operator-(const DQ& dq, const int& scalar);
DQ operator-(const int& scalar, const DQ& dq);
DQ operator-(const DQ& dq, const float &scalar);
DQ operator-(const float &scalar, const DQ& dq);
DQ operator-(const DQ& dq, const double& scalar);
DQ operator-(const double& scalar, const DQ& dq);

//Operator (*) Overload
DQ operator*(const DQ& dq1, const DQ& dq2);
DQ operator*(const DQ& dq, const int& scalar);
DQ operator*(int& scalar, const DQ& dq);
DQ operator*(const DQ& dq, const float &scalar);
DQ operator*(const float &scalar, const DQ& dq);
DQ operator*(const DQ& dq, const double& scalar);
DQ operator*(const double& scalar, const DQ& dq);

//Operator (==) Overload
bool operator==(const DQ& dq, const int& scalar);
bool operator==(const int& scalar, const DQ& dq);
bool operator==(const DQ& dq, const float &scalar);
bool operator==(const float &scalar, const DQ& dq);
bool operator==(const DQ& dq, const double& scalar);
bool operator==(const double& scalar, const DQ& dq);

//Operator (!=) Overload
bool operator!=(const DQ& dq, const int& scalar);
bool operator!=(const int& scalar, const DQ& dq);
bool operator!=(const DQ& dq, const float &scalar);
bool operator!=(const float &scalar, const DQ& dq);
bool operator!=(const DQ& dq, const double& scalar);
bool operator!=(const double& scalar, const DQ& dq);

//Operator (<<) Overload
std::ostream& operator<<(std::ostream &os, const DQ& dq);

//Constants
Matrix<double,8,8> C8();
Matrix<double,4,4> C4();

const DQ E_ = DQ(0,0,0,0,1,0,0,0);
const DQ i_ = DQ(0,1,0,0,0,0,0,0);
const DQ j_ = DQ(0,0,1,0,0,0,0,0);
const DQ k_ = DQ(0,0,0,1,0,0,0,0);

const double DQ_threshold = 1e-12;

//Operators
DQ P(const DQ& dq);

DQ D(const DQ& dq);

DQ Re(const DQ& dq);

DQ Im(const DQ& dq);

DQ conj(const DQ& dq);

DQ norm(const DQ& dq);

DQ inv(const DQ& dq);

DQ translation(const DQ& dq);

DQ rotation(const DQ& dq);

DQ rotation_axis(const DQ& dq);

double rotation_angle(const DQ& dq);

DQ log(const DQ& dq);

DQ exp(const DQ& dq);

DQ pow(const DQ& dq, const double& a);

DQ tplus(const DQ& dq);
inline DQ T(const DQ& dq){return tplus(dq);}

DQ pinv(const DQ& dq);

DQ dec_mult(const DQ& dq1, const DQ& dq2);

Matrix4d hamiplus4(const DQ& dq);

Matrix4d haminus4(const DQ& dq);

Matrix<double,8,8> hamiplus8(const DQ& dq);

Matrix<double,8,8> haminus8(const DQ& dq);

Matrix<double,8,8> generalized_jacobian(const DQ& dq);

Vector4d vec4(const DQ& dq);

Matrix<double,8,1> vec8(const DQ& dq);

Matrix4d crossmatrix4(const DQ& dq);

DQ normalize (const DQ& dq);

DQ sharp(const DQ& dq);

DQ line(const DQ& l, const DQ& pl);

DQ plane(const DQ& n, const DQ& ppi);

DQ cross(const DQ& a, const DQ& b);

DQ dot(const DQ& a, const DQ& b);

///Deprecated signatures
DEPRECATED Matrix4d Hplus4(const DQ& dq);
DEPRECATED Matrix4d Hminus4(const DQ& dq);
DEPRECATED Matrix<double,8,8> Hplus8(const DQ& dq);
DEPRECATED Matrix<double,8,8> Hminus8(const DQ& dq);
DEPRECATED Matrix<double,8,8> generalizedJacobian(const DQ& dq);
DEPRECATED Matrix<double,8,8> jacobG(const DQ& dq);
DEPRECATED double rot_angle(const DQ& dq);
DEPRECATED DQ rot_axis(const DQ& dq);

}//Namespace DQRobotics

#endif // DQ_H
