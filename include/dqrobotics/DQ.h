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

/**
 * @brief Units, operations, and operators related to dual quaternions.
 *
 * A dual quaternion is represented internally as an eight-dimensional
 * vector `q`, ordered as (real part, imaginary i, j, k, and dual part
 * real, i, j, k components). A dual quaternion can be constructed from the
 * dual unit `DQ::E` and the imaginary units `DQ::i`, `DQ::j`, `DQ::k`
 * (e.g., `DQ dq = 1 + 2*DQ::i + 5*DQ::E*DQ::k;`), or directly from a vector
 * of 8, 6, 4, 3, or 1 dimensions containing the dual quaternion
 * coefficients:
 *   - An eight-dimensional vector contains the coefficients of a general
 *     dual quaternion.
 *   - A six-dimensional vector contains the coefficients of a pure dual
 *     quaternion.
 *   - A four-dimensional vector contains the coefficients of a general
 *     quaternion.
 *   - A three-dimensional vector contains the coefficients of a pure
 *     quaternion.
 *   - A one-dimensional vector contains the coefficient of a real number.
 */
class DQ{

private:
    VectorXd q_() const;
    double q_(const int a) const;

public:
    //Member
    Matrix<double,8,1> q; ///< The eight-dimensional vector of coefficients of this dual quaternion.

    //Static
    /**
     * @brief Builds the unit dual quaternion representing a rotation of
     * @p rot_angle radians around the axis (@p x_axis, @p y_axis, @p z_axis)
     * combined with the translation (@p x_trans, @p y_trans, @p z_trans).
     * @param rot_angle The rotation angle, in radians.
     * @param x_axis The x component of the rotation axis (pure quaternion).
     * @param y_axis The y component of the rotation axis (pure quaternion).
     * @param z_axis The z component of the rotation axis (pure quaternion).
     * @param x_trans The x component of the translation.
     * @param y_trans The y component of the translation.
     * @param z_trans The z component of the translation.
     * @return The corresponding unit dual quaternion.
     */
    static DQ unitDQ( const double& rot_angle,
                      const int& x_axis,
                      const int& y_axis,
                      const int& z_axis,
                      const double& x_trans,
                      const double& y_trans,
                      const double& z_trans);
    //To comply with MATLAB
    const static DQ i; ///< The imaginary unit i.
    const static DQ j; ///< The imaginary unit j.
    const static DQ k; ///< The imaginary unit k.
    const static DQ E; ///< The dual unit.

    //Constructors
    /// @brief Constructs a dual quaternion from a vector of 8, 6, 4, 3, or 1
    /// dimensions containing the dual quaternion coefficients.
    explicit DQ(VectorXd&& v);
    /**
     * @brief Constructs a dual quaternion from a vector of 8, 6, 4, 3, or 1
     * dimensions containing the dual quaternion coefficients.
     * @param v The vector containing the dual quaternion coefficients.
     */
    explicit DQ(const VectorXd& v);

    /**
     * @brief Constructs a dual quaternion from its eight coefficients.
     * @param q0 Real part of the real component.
     * @param q1 Coefficient of the imaginary unit i of the real component.
     * @param q2 Coefficient of the imaginary unit j of the real component.
     * @param q3 Coefficient of the imaginary unit k of the real component.
     * @param q4 Real part of the dual component.
     * @param q5 Coefficient of the imaginary unit i of the dual component.
     * @param q6 Coefficient of the imaginary unit j of the dual component.
     * @param q7 Coefficient of the imaginary unit k of the dual component.
     */
    explicit DQ(const double& q0=0.0,
                const double& q1=0.0,
                const double& q2=0.0,
                const double& q3=0.0,
                const double& q4=0.0,
                const double& q5=0.0,
                const double& q6=0.0,
                const double& q7=0.0) noexcept;

    //Member functions
    /// @brief Returns the primary part of this dual quaternion.
    DQ P() const;

    /// @brief Returns the dual part of this dual quaternion.
    DQ D() const;

    /// @brief Returns the real part of this dual quaternion.
    DQ Re() const;

    /// @brief Returns the imaginary part of this dual quaternion.
    DQ Im() const;

    /**
     * @brief Returns the conjugate of this dual quaternion.
     * @see operator~(), transpose()
     */
    DQ conj() const;

    /// @brief Returns the dual scalar corresponding to the norm of this dual quaternion.
    DQ norm() const;

    /**
     * @brief Returns the inverse of this dual quaternion, given by
     * `conj()/(norm()^2)`.
     * @see pinv()
     */
    DQ inv() const;

    /**
     * @brief Returns the translation quaternion of this unit dual
     * quaternion, assuming `dq = r + DQ::E * 0.5 * p * r`, that is, the
     * translation followed by rotation motion.
     * @return The pure quaternion `p` representing the translation.
     */
    DQ translation() const;

    /**
     * @brief Returns the rotation quaternion of this unit dual quaternion.
     * @throws std::runtime_error if this dual quaternion does not have unit norm.
     */
    DQ rotation() const;

    /**
     * @brief Returns the rotation axis of this unit dual quaternion.
     * @return The pure quaternion representing the rotation axis
     * (`nx*i + ny*j + nz*k`).
     * @note If the rotation angle is zero, the axis is not well defined and,
     * by convention, the axis `k` is returned.
     * @throws std::runtime_error if this dual quaternion does not have unit norm.
     */
    DQ rotation_axis() const;

    /**
     * @brief Returns the rotation angle of this unit dual quaternion.
     * @throws std::runtime_error if this dual quaternion does not have unit norm.
     */
    double rotation_angle() const;

    /// @brief Returns the logarithm of this dual quaternion.
    DQ log() const;

    /// @brief Returns the exponential of this pure dual quaternion.
    DQ exp() const;

    /**
     * @brief Returns this dual quaternion raised to the power of @p a.
     * @param a The exponent.
     */
    DQ pow(const double a) const;

    /**
     * @brief Returns the unit dual quaternion corresponding to the
     * translation part of this dual quaternion. More specifically, if
     * `dq = r + DQ::E*0.5*p*r`, `tplus()` returns `1 + DQ::E*0.5*p`.
     */
    DQ tplus() const;
    /// @brief Alias for tplus().
    inline DQ T() const{return tplus();}

    /// @brief Returns the Moore-Penrose pseudoinverse of this dual quaternion.
    DQ pinv() const;

    /// @brief Returns the Hamilton operator H+ of this dual quaternion, restricted to its primary part.
    Matrix4d hamiplus4() const;

    /// @brief Returns the Hamilton operator H- of this dual quaternion, restricted to its primary part.
    Matrix4d haminus4() const;

    /// @brief Returns the Hamilton operator H+ of this dual quaternion.
    Matrix<double,8,8> hamiplus8() const;

    /// @brief Returns the Hamilton operator H- of this dual quaternion.
    Matrix<double,8,8> haminus8() const;

    /// @brief Maps the primary part of this dual quaternion into a 3-dimensional vector.
    Vector3d vec3() const;

    /// @brief Maps the primary part of this dual quaternion into a 4-dimensional vector.
    Vector4d vec4() const;

    /// @brief Maps this dual quaternion into a 6-dimensional vector, discarding the real part of both the primary and dual components.
    Matrix<double,6,1> vec6() const;

    /// @brief Maps this dual quaternion into an 8-dimensional vector.
    Matrix<double,8,1> vec8() const;

    /// @brief Returns the generalized Jacobian used in the mapping between the time derivative of a unit dual quaternion and the twist it represents.
    Matrix<double,8,8> generalized_jacobian() const;

    /// @brief Returns this dual quaternion normalized to unit norm.
    DQ normalize() const;

    /**
     * @brief Returns the sharp conjugate of this dual quaternion.
     * @see operator~()
     */
    DQ sharp() const;

    /**
     * @brief Returns the adjoint transformation `this * dq2 * this'`.
     *
     * Given a Plücker line represented by the pure dual quaternion @p dq2,
     * expressed with respect to the frame represented by this unit dual
     * quaternion, `Ad()` returns the Plücker line expressed in the base frame.
     * @param dq2 The pure dual quaternion representing a Plücker line.
     * @see Adsharp()
     */
    DQ Ad(const DQ& dq2) const;

    /**
     * @brief Returns the sharp adjoint transformation `this.sharp() * dq2 * this'`.
     *
     * Given a plane represented by the dual quaternion @p dq2, expressed with
     * respect to the frame represented by this unit dual quaternion,
     * `Adsharp()` returns the plane expressed in the base frame.
     * @param dq2 The dual quaternion representing a plane.
     * @see Ad()
     */
    DQ Adsharp(const DQ& dq2) const;

    /**
     * @brief Given the unit quaternion represented by this dual quaternion,
     * returns the partial derivative of `vec4()` with respect to
     * `vec3(log())`.
     *
     * See Eq. (22) of Savino et al. (2020), "Pose consensus based on dual
     * quaternion algebra with application to decentralized formation control
     * of mobile manipulators." https://doi.org/10.1016/j.jfranklin.2019.09.045
     */
    Matrix<double,4,3> Q4() const;

    /**
     * @brief Given this unit dual quaternion, returns the partial derivative
     * of `vec8()` with respect to `vec6(log())`.
     *
     * See Theorem 4 of Savino et al. (2020), "Pose consensus based on dual
     * quaternion algebra with application to decentralized formation control
     * of mobile manipulators." https://doi.org/10.1016/j.jfranklin.2019.09.045
     */
    Matrix<double,8,6> Q8() const;

    /// @brief Returns a string representation of this dual quaternion.
    std::string to_string() const;

    //Operators
    /// @brief Returns the additive inverse of this dual quaternion.
    DQ operator-() const;
    /// @brief Returns true if this dual quaternion and @p dq2 are equal, up to a numerical threshold.
    bool operator==(const DQ& dq2) const;
    /// @brief Returns true if this dual quaternion and @p dq2 are different, up to a numerical threshold.
    bool operator!=(const DQ& dq2) const;
    /**
     * @brief Casts this dual quaternion to a `double`.
     * @throws std::runtime_error if this dual quaternion does not represent a real number.
     */
    explicit operator double() const;
    /**
     * @brief Casts this dual quaternion to an `int`.
     * @throws std::runtime_error if this dual quaternion does not represent a real number.
     */
    explicit operator int()    const;

    //Assigment operator template for scalars
    /// @brief Assigns the scalar @p s to the real part of this dual quaternion, setting all remaining coefficients to zero.
    template <typename Scalar, typename scalar = std::enable_if<std::is_arithmetic<Scalar>::value>>
    DQ& operator=(const Scalar& s)
    {
        q = VectorXd::Zero(8);
        q(0)=s;
        return *this;
    };
};//DQ Class END

//Operators
/// @brief Returns the primary part of @p dq.
DQ P(const DQ& dq);

/// @brief Returns the dual part of @p dq.
DQ D(const DQ& dq);

/// @brief Returns the real part of @p dq.
DQ Re(const DQ& dq);

/// @brief Returns the imaginary part of @p dq.
DQ Im(const DQ& dq);

/// @brief Returns the conjugate of @p dq.
DQ conj(const DQ& dq);

/// @brief Returns the dual scalar corresponding to the norm of @p dq.
DQ norm(const DQ& dq);

/**
 * @brief Returns the inverse of @p dq, given by `conj(dq)/(norm(dq)^2)`.
 * @see pinv(const DQ&)
 */
DQ inv(const DQ& dq);

/**
 * @brief Returns the translation quaternion of the unit dual quaternion @p dq,
 * assuming `dq = r + DQ::E*0.5*p*r`.
 */
DQ translation(const DQ& dq);

/**
 * @brief Returns the rotation quaternion of the unit dual quaternion @p dq.
 * @throws std::runtime_error if @p dq does not have unit norm.
 */
DQ rotation(const DQ& dq);

/**
 * @brief Returns the rotation axis (`nx*i + ny*j + nz*k`) of the unit dual
 * quaternion @p dq.
 * @note If the rotation angle is zero, the axis is not well defined and,
 * by convention, the axis `k` is returned.
 * @throws std::runtime_error if @p dq does not have unit norm.
 */
DQ rotation_axis(const DQ& dq);

/**
 * @brief Returns the rotation angle of the unit dual quaternion @p dq.
 * @throws std::runtime_error if @p dq does not have unit norm.
 */
double rotation_angle(const DQ& dq);

/// @brief Returns the logarithm of the dual quaternion @p dq.
DQ log(const DQ& dq);

/// @brief Returns the exponential of the pure dual quaternion @p dq.
DQ exp(const DQ& dq);

/**
 * @brief Returns @p dq raised to the power of @p a.
 * @param dq The dual quaternion base.
 * @param a The exponent.
 */
DQ pow(const DQ& dq, const double& a);

/**
 * @brief Returns the unit dual quaternion corresponding to the translation
 * part of @p dq. More specifically, if `dq = r + DQ::E*0.5*p*r`, `tplus()`
 * returns `1 + DQ::E*0.5*p`.
 */
DQ tplus(const DQ& dq);
/// @brief Alias for tplus(const DQ&).
inline DQ T(const DQ& dq){return tplus(dq);}

/// @brief Returns the Moore-Penrose pseudoinverse of @p dq.
DQ pinv(const DQ& dq);

/**
 * @brief Returns the decompositional multiplication between @p dq1 and @p dq2.
 * @param dq1 The first dual quaternion.
 * @param dq2 The second dual quaternion.
 */
DQ dec_mult(const DQ& dq1, const DQ& dq2);

/// @brief Returns the Hamilton operator H+ of @p dq, restricted to its primary part.
Matrix4d hamiplus4(const DQ& dq);

/// @brief Returns the Hamilton operator H- of @p dq, restricted to its primary part.
Matrix4d haminus4(const DQ& dq);

/// @brief Returns the Hamilton operator H+ of @p dq.
Matrix<double,8,8> hamiplus8(const DQ& dq);

/// @brief Returns the Hamilton operator H- of @p dq.
Matrix<double,8,8> haminus8(const DQ& dq);

/// @brief Returns the generalized Jacobian used in the mapping between the time derivative of the unit dual quaternion @p dq and the twist it represents.
Matrix<double,8,8> generalized_jacobian(const DQ& dq);

/// @brief Maps the primary part of @p dq into a 3-dimensional vector.
Vector3d vec3(const DQ& dq);

/// @brief Maps the primary part of @p dq into a 4-dimensional vector.
Vector4d vec4(const DQ& dq);

/// @brief Maps @p dq into a 6-dimensional vector, discarding the real part of both the primary and dual components.
Matrix<double,6,1> vec6(const DQ& dq);

/// @brief Maps @p dq into an 8-dimensional vector.
Matrix<double,8,1> vec8(const DQ& dq);

/**
 * @brief Maps the pure quaternion @p dq into an expanded skew-symmetric matrix
 * such that `vec4(cross(dq,v)) = crossmatrix4(dq)*vec4(v)`.
 */
Matrix4d crossmatrix4(const DQ& dq);

/// @brief Returns @p dq normalized to unit norm.
DQ normalize (const DQ& dq);

/**
 * @brief Returns the sharp conjugate of @p dq.
 */
DQ sharp(const DQ& dq);

/// @brief Returns the cross product between the pure dual quaternions @p dq1 and @p dq2.
DQ cross(const DQ& dq1, const DQ& dq2);

/// @brief Returns the dot product between the pure dual quaternions @p dq1 and @p dq2.
DQ dot(const DQ& dq1, const DQ& dq2);

/**
 * @brief Returns the adjoint transformation `dq1 * dq2 * dq1'`.
 *
 * Given a Plücker line represented by the pure dual quaternion @p dq2,
 * expressed with respect to the frame represented by the unit dual
 * quaternion @p dq1, `Ad()` returns the Plücker line expressed in the base
 * frame.
 * @see Adsharp(const DQ&, const DQ&)
 */
DQ Ad(const DQ& dq1, const DQ& dq2);

/**
 * @brief Returns the sharp adjoint transformation `sharp(dq1) * dq2 * dq1'`.
 *
 * Given a plane represented by the dual quaternion @p dq2, expressed with
 * respect to the frame represented by the unit dual quaternion @p dq1,
 * `Adsharp()` returns the plane expressed in the base frame.
 * @see Ad(const DQ&, const DQ&)
 */
DQ Adsharp(const DQ& dq1, const DQ& dq2);

/**
 * @brief Given the unit quaternion @p dq, returns the partial derivative of
 * `vec4(dq)` with respect to `vec3(log(dq))`.
 *
 * See Eq. (22) of Savino et al. (2020), "Pose consensus based on dual
 * quaternion algebra with application to decentralized formation control of
 * mobile manipulators." https://doi.org/10.1016/j.jfranklin.2019.09.045
 */
Matrix<double,4,3> Q4(const DQ& dq);

/**
 * @brief Given the unit dual quaternion @p dq, returns the partial derivative
 * of `vec8(dq)` with respect to `vec6(log(dq))`.
 *
 * See Theorem 4 of Savino et al. (2020), "Pose consensus based on dual
 * quaternion algebra with application to decentralized formation control of
 * mobile manipulators." https://doi.org/10.1016/j.jfranklin.2019.09.045
 */
Matrix<double,8,6> Q8(const DQ& dq);

/**
 * @brief Returns true if @p dq is a unit norm dual quaternion, false otherwise.
 * @see is_pure(const DQ&), is_quaternion(const DQ&), is_real(const DQ&), is_real_number(const DQ&)
 */
bool is_unit(const DQ& dq);

/**
 * @brief Returns true if @p dq is pure (i.e., `Re(dq) = 0`), false otherwise.
 * @see is_line(const DQ&), is_plane(const DQ&), is_pure_quaternion(const DQ&), is_quaternion(const DQ&), is_real(const DQ&), is_real_number(const DQ&), is_unit(const DQ&)
 */
bool is_pure(const DQ& dq);

/**
 * @brief Returns true if the imaginary part of @p dq is zero, false otherwise.
 * @note A real dual quaternion is not necessarily a strict real number because
 * it can also be a dual number.
 * @see is_pure(const DQ&), is_quaternion(const DQ&), is_real_number(const DQ&), is_unit(const DQ&)
 */
bool is_real(const DQ& dq);

/**
 * @brief Returns true if both the dual and imaginary parts of @p dq are zero, false otherwise.
 * @see is_pure(const DQ&), is_quaternion(const DQ&), is_real(const DQ&), is_unit(const DQ&)
 */
bool is_real_number(const DQ& dq);

/**
 * @brief Returns true if the dual part of @p dq is zero, false otherwise.
 * @see is_pure(const DQ&), is_real(const DQ&), is_real_number(const DQ&), is_unit(const DQ&)
 */
bool is_quaternion(const DQ& dq);

/**
 * @brief Returns true if @p dq is a pure quaternion (i.e., `Re(dq) = D(dq) = 0`), false otherwise.
 * @see is_quaternion(const DQ&), is_pure(const DQ&), is_real(const DQ&), is_real_number(const DQ&), is_unit(const DQ&)
 */
bool is_pure_quaternion(const DQ& dq);

/**
 * @brief Returns true if @p dq is a line (i.e., `Re(dq) = 0` and `norm(dq) = 1`), false otherwise.
 * @see is_plane(const DQ&), is_pure(const DQ&), is_pure_quaternion(const DQ&), is_quaternion(const DQ&), is_real(const DQ&), is_real_number(const DQ&), is_unit(const DQ&)
 */
bool is_line(const DQ& dq);

/**
 * @brief Returns true if @p dq is a plane (i.e., it has unit norm and `Im(D(dq)) = 0`), false otherwise.
 * @see is_pure(const DQ&), is_pure_quaternion(const DQ&), is_quaternion(const DQ&), is_real(const DQ&), is_real_number(const DQ&), is_unit(const DQ&)
 */
bool is_plane(const DQ& dq);

/*************************************************************************
 ************** DUAL QUATERNION CONSTANTS AND OPERATORS ******************
 ************************************************************************/

/// @brief Numerical threshold used to compare dual quaternions for equality.
constexpr double DQ_threshold = 1e-12;

/// @brief Returns the dual quaternion addition `dq1 + dq2`.
const DQ operator+(const DQ& dq1, const DQ& dq2) noexcept;
/// @brief Rvalue-reference overload of operator+(const DQ&, const DQ&) that reuses @p rdq1 to avoid a copy.
const DQ operator+(DQ&& rdq1, const DQ& dq2) noexcept;
/// @brief Rvalue-reference overload of operator+(const DQ&, const DQ&) that reuses @p rdq2 to avoid a copy.
const DQ operator+(const DQ& dq1, DQ&& rdq2) noexcept;
/// @brief Rvalue-reference overload of operator+(const DQ&, const DQ&) that reuses @p rdq1 to avoid a copy.
const DQ operator+(DQ&& rdq1, DQ&& rdq2) noexcept;

/// @brief Returns the dual quaternion subtraction `dq1 - dq2`.
const DQ operator-(const DQ& dq1, const DQ& dq2) noexcept;
/// @brief Rvalue-reference overload of operator-(const DQ&, const DQ&) that reuses @p rdq1 to avoid a copy.
const DQ operator-(DQ&& rdq1, const DQ& dq2) noexcept;
//const DQ operator-(const DQ& dq1, DQ&& rdq2) noexcept; //TODO: Think of a smart way to implement this
/// @brief Rvalue-reference overload of operator-(const DQ&, const DQ&) that reuses @p rdq1 to avoid a copy.
const DQ operator-(DQ&& rdq1, DQ&& rdq2) noexcept;

/// @brief Returns the dual quaternion multiplication `dq1 * dq2`.
const DQ operator*(const DQ& dq1, const DQ& dq2) noexcept;

//Operator (<<) Overload
/// @brief Streams a string representation of @p dq into @p os.
std::ostream& operator<<(std::ostream &os, const DQ& dq);

//Constants
/// @brief Returns the conjugator matrix associated with vec8().
Matrix<double,8,8> C8();
/// @brief Returns the conjugator matrix associated with vec4().
Matrix<double,4,4> C4();

const DQ E_ = DQ(0,0,0,0,1,0,0,0); ///< Shortcut for the dual unit DQ::E.
const DQ i_ = DQ(0,1,0,0,0,0,0,0); ///< Shortcut for the imaginary unit DQ::i.
const DQ j_ = DQ(0,0,1,0,0,0,0,0); ///< Shortcut for the imaginary unit DQ::j.
const DQ k_ = DQ(0,0,0,1,0,0,0,0); ///< Shortcut for the imaginary unit DQ::k.

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
