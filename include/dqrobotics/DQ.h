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

3. Murilo M. Marinho (murilomarinho@ieee.org)
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
#pragma once

#include <Eigen/Dense>

#include <iostream>
using namespace Eigen;

namespace DQ_robotics{


class DQ{

private:
    VectorXd q_() const;
    double q_(const int a) const;

public:
    //Member
    Matrix<double,8,1> q;

    //Static
    static DQ unitDQ( const double& rot_angle,
                      const int& x_axis,
                      const int& y_axis,
                      const int& z_axis,
                      const double& x_trans,
                      const double& y_trans,
                      const double& z_trans);
    //To comply with MATLAB
    const static DQ i;
    const static DQ j;
    const static DQ k;
    const static DQ E;

    //Constructors
    explicit DQ(VectorXd&& v);
    explicit DQ(const VectorXd& v);

    explicit DQ(const double& q0=0.0,
                const double& q1=0.0,
                const double& q2=0.0,
                const double& q3=0.0,
                const double& q4=0.0,
                const double& q5=0.0,
                const double& q6=0.0,
                const double& q7=0.0) noexcept;

    //Member functions
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

    Vector3d vec3() const;

    Vector4d vec4() const;

    Matrix<double,6,1> vec6() const;

    Matrix<double,8,1> vec8() const;

    Matrix<double,8,8> generalized_jacobian() const;

    DQ normalize() const;

    DQ sharp() const;

    DQ Ad(const DQ& dq2) const;

    DQ Adsharp(const DQ& dq2) const;

    Matrix<double,4,3> Q4() const;

    Matrix<double,8,6> Q8() const;

    std::string to_string() const;

    //Operators
    DQ operator-() const;
    bool operator==(const DQ& dq2) const;
    bool operator!=(const DQ& dq2) const;
    explicit operator double() const;
    explicit operator int()    const;

    //Assigment operator template for scalars
    template <typename Scalar, typename scalar = std::enable_if<std::is_arithmetic<Scalar>::value>>
    DQ& operator=(const Scalar& s)
    {
        q = VectorXd::Zero(8);
        q(0)=s;
        return *this;
    };
};//DQ Class END

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

Vector3d vec3(const DQ& dq);

Vector4d vec4(const DQ& dq);

Matrix<double,6,1> vec6(const DQ& dq);

Matrix<double,8,1> vec8(const DQ& dq);

Matrix4d crossmatrix4(const DQ& dq);

DQ normalize (const DQ& dq);

DQ sharp(const DQ& dq);

DQ cross(const DQ& dq1, const DQ& dq2);

DQ dot(const DQ& dq1, const DQ& dq2);

DQ Ad(const DQ& dq1, const DQ& dq2);

DQ Adsharp(const DQ& dq1, const DQ& dq2);

Matrix<double,4,3> Q4(const DQ& dq);

Matrix<double,8,6> Q8(const DQ& dq);

bool is_unit(const DQ& dq);

bool is_pure(const DQ& dq);

bool is_real(const DQ& dq);

bool is_real_number(const DQ& dq);

bool is_quaternion(const DQ& dq);

bool is_pure_quaternion(const DQ& dq);

bool is_line(const DQ& dq);

bool is_plane(const DQ& dq);

/*************************************************************************
 ************** DUAL QUATERNION CONSTANTS AND OPERATORS ******************
 ************************************************************************/

constexpr double DQ_threshold = 1e-12;

const DQ operator+(const DQ& dq1, const DQ& dq2) noexcept;
const DQ operator+(DQ&& rdq1, const DQ& dq2) noexcept;
const DQ operator+(const DQ& dq1, DQ&& rdq2) noexcept;
const DQ operator+(DQ&& rdq1, DQ&& rdq2) noexcept;

const DQ operator-(const DQ& dq1, const DQ& dq2) noexcept;
const DQ operator-(DQ&& rdq1, const DQ& dq2) noexcept;
//const DQ operator-(const DQ& dq1, DQ&& rdq2) noexcept; //TODO: Think of a smart way to implement this
const DQ operator-(DQ&& rdq1, DQ&& rdq2) noexcept;

const DQ operator*(const DQ& dq1, const DQ& dq2) noexcept;

//Operator (<<) Overload
std::ostream& operator<<(std::ostream &os, const DQ& dq);

//Constants
Matrix<double,8,8> C8();
Matrix<double,4,4> C4();

const DQ E_ = DQ(0,0,0,0,1,0,0,0);
const DQ i_ = DQ(0,1,0,0,0,0,0,0);
const DQ j_ = DQ(0,0,1,0,0,0,0,0);
const DQ k_ = DQ(0,0,0,1,0,0,0,0);

/*************************************************************************
 ************** DUAL QUATERNIONS AND SCALAR OPERATOR TEMPLATES ***********
 ************************************************************************/

template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
/**
 * @brief operator + between a DQ and a Scalar.
 * @param dq a DQ.
 * @param s any Scalar (as defined by std::is_arithmetic, bools are purposedly removed using static_assert.)
 * @return the sum between the DQ and the Scalar.
 */
inline const DQ operator+(const DQ& dq, const Scalar& s) noexcept
{
    static_assert(!std::is_same<bool,Scalar>(),"Operations between DQs and bools are not supported.");

    DQ ret(dq);
    ret.q(0)+=s;
    return ret;
}

template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
/**
 * @brief operator + between a DQ and a Scalar.
 * @param s any Scalar (as defined by std::is_arithmetic, bools are purposedly removed using static_assert.)
 * @param dq a DQ.
 * @return the sum between the DQ and the Scalar.
 */
inline const DQ operator+(const Scalar& s, const DQ& dq) noexcept
{
    static_assert(!std::is_same<bool,Scalar>(),"Operations between DQs and bools are not supported.");

    DQ ret(dq);
    ret.q(0)+=s;
    return ret;
}

template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
/**
 * @brief operator - between a DQ and a Scalar.
 * @param dq a DQ.
 * @param s any Scalar (as defined by std::is_arithmetic, bools are purposedly removed using static_assert.)
 * @return the difference between the DQ and the Scalar.
 */
inline const DQ operator-(const DQ& dq, const Scalar& s) noexcept
{
    static_assert(!std::is_same<bool,Scalar>(),"Operations between DQs and bools are not supported.");

    DQ ret(dq);
    ret.q(0)-=s;
    return ret;
}

template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
/**
 * @brief operator - between a DQ and a Scalar.
 * @param s any Scalar (as defined by std::is_arithmetic, bools are purposedly removed using static_assert.)
 * @param dq a DQ.
 * @return the difference between the DQ and the Scalar.
 */
inline const DQ operator-(const Scalar& s, const DQ& dq) noexcept
{
    static_assert(!std::is_same<bool,Scalar>(),"Operations between DQs and bools are not supported.");

    DQ ret(dq);
    ret.q(0)-=s;
    return ret;
}

template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
/**
 * @brief operator * between a DQ and a Scalar.
 * @param dq a DQ.
 * @param s any Scalar (as defined by std::is_arithmetic, bools are purposedly removed using static_assert.)
 * @return the product between the DQ and the Scalar.
 */
inline const DQ operator*(const DQ& dq, const Scalar& s) noexcept
{
    static_assert(!std::is_same<bool,Scalar>(),"Operations between DQs and bools are not supported.");

    return DQ(s*dq.q);
}

template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
/**
 * @brief operator * between a DQ and a Scalar.
 * @param s any Scalar (as defined by std::is_arithmetic, bools are purposedly removed using static_assert.)
 * @param dq a DQ.
 * @return the product between the DQ and the Scalar.
 */
inline const DQ operator*(const Scalar& s, const DQ& dq) noexcept
{
    static_assert(!std::is_same<bool,Scalar>(),"Operations between DQs and bools are not supported.");

    return DQ(s*dq.q);
}


template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
/**
 * @brief __dq_scalar_equal_impl Implementation of the operator == between a DQ and a Scalar,
 * shared between == and != operators.
 * @param dq a DQ.
 * @param s any Scalar (as defined by std::is_arithmetic, bools are purposedly removed using static_assert.)
 * @return true if the DQ and Scalar are equal (up to the DQ_treshold), false otherwise.
 */
inline bool __dq_scalar_equal_impl(const DQ& dq, const Scalar& s) noexcept
{
    static_assert(!std::is_same<bool,Scalar>(),"Operations between DQs and bools are not supported.");

    if(fabs(dq.q(0)-s)>DQ_threshold)
        return false;
    for(auto i=1;i<8;i++)
    {
        if(fabs(dq.q(i)-0.0)>DQ_threshold)
            return false;
    }
    return true;
}


template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
/**
 * @brief operator == between a DQ and a Scalar.
 * @param dq a DQ.
 * @param s any Scalar (as defined by std::is_arithmetic, bools are purposedly removed using static_assert.)
 * @return true if the DQ and Scalar are equal (up to the DQ_treshold), false otherwise.
 */
inline bool operator==(const DQ& dq, const Scalar& s) noexcept
{
    return __dq_scalar_equal_impl(dq,s);
}

template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
/**
 * @brief operator == between a DQ and a Scalar.
 * @param s any Scalar (as defined by std::is_arithmetic, bools are purposedly removed using static_assert.)
 * @param dq a DQ.
 * @return true if the DQ and Scalar are equal (up to the DQ_treshold), false otherwise.
 */
inline bool operator==(const Scalar& s, const DQ& dq) noexcept
{
    return __dq_scalar_equal_impl(dq,s);
}

template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
/**
 * @brief operator != between a DQ and a Scalar.
 * @param dq a DQ.
 * @param s any Scalar (as defined by std::is_arithmetic, bools are purposedly removed using static_assert.)
 * @return true if the DQ and Scalar are different (up to the DQ_treshold), false otherwise.
 */
inline bool operator!=(const DQ& dq, const Scalar& s) noexcept
{
    return !__dq_scalar_equal_impl(dq,s);
}

template <typename Scalar, typename = typename std::enable_if<std::is_arithmetic<Scalar>::value>::type>
/**
 * @brief operator != between a DQ and a Scalar.
 * @param s any Scalar (as defined by std::is_arithmetic, bools are purposedly removed using static_assert.)
 * @param dq a DQ.
 * @return true if the DQ and Scalar are different (up to the DQ_treshold), false otherwise.
 */
inline bool operator!=(const Scalar& s, const DQ& dq) noexcept
{
    return !__dq_scalar_equal_impl(dq,s);
}


}
