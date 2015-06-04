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

/**
* This class DQ represents a Dual Quaternion.
*
* In the class definition are declared different constructors for the Dual Quaternion object, the public methods which can be called
* by the object, the operators overload functions and also auxiliar functions and variables to intermediate the operations of the
* public methods. Most of the methods returns a constant Dual Quaternion object, which depends of the object caller such as primary
* and dual parts or not, being the same for any caller such as the imaginary parts. Some methods returns a constant boost matrix class
* object which depends of object caller too. And there is a method for display in the console, the DQ object caller.

***********************************************************
*              REVISION HISTORY
***********************************************************
* 2015/5/26 Murilo Marques Marinho (murilo@nml.t.u-tokyo.ac.jp)
             - Added functions to allow compatibility between C++
               and MATLAB code. (haminus4,8 hamiplus4,8 T(), 
               rotation_axis(), rotation_angle().

* 2014/10/15 Murilo Marques Marinho (murilomarinho@lara.unb.br)
             - Fixed all implementations of jacobG and 
               generalizedJacobian so that they make sense.

* 2013/11/20 Murilo Marques Marinho (murilomarinho@lara.unb.br)
             - Removed the last redundant constructor.

* 2013/11/19 Murilo Marques Marinho (murilomarinho@lara.unb.br)
             - Removed redundant constructors and used de
               fault value constructors.
             - Added range_error thrown() in the arithmetic
               functions that require unit quaternions as
               inputs.
             - Removed unecessary copying of objects in those 
               functions.
             - Removed unecessary includes.

* 2013/11/18 Murilo Marques Marinho (murilomarinho@lara.unb.br)
             - Added 
               - const qualifiers to specify when the arguments
                 are changed by the function.
               - const qualifiers to specify when a method
                 changes the object
               - argument references in place of copies whenever
                 possible.
             - Note that the operator overloading methods were
               not changed. 

* 2013/11/15 Murilo Marques Marinho (murilomarinho@lara.unb.br)
             - Changed << operator from reference to argument
               copy. 

* 2013/11/14 Murilo Marques Marinho (murilomarinho@lara.unb.br)
             - Removed qualifiers that were only causing
               warnings.

* 2013/31/01 Murilo Marques Marinho (murilomarinho@lara.unb.br)
             - Changed Library to Use Eigen.

* 2013/07/02 Murilo Marques Marinho (murilomarinho@lara.unb.br)
             - Removed static functions and created DQRobotics
               namespace.
             - Some static methods became constants:
                 - E() -> dual_E
                 - i() -> im_i
                 - j() -> im_j
                 - k() -> im_k
                 - threshold() -> DQ_threshold
             - Created generalizedJacobian() method but kept
               the method with MATLAB syntax for legacy reasons.
             - Removed "DISPLAY" method and overloaded the "<<"
               operator in order to use with cout.

* 2012/12/10 Mateus Rodrigues Martins (martinsrmateus@gmail.com)
             - First working version implementing MATLAB
               functionality
***********************************************************
*/

#ifndef DQ_H
#define DQ_H

#include <iostream>
#include <math.h>
#include <stdexcept> //for range_error
#include <Eigen/Dense>
using namespace Eigen;

namespace DQ_robotics{


    class DQ{

        //Atributes
      public:

      Matrix<double,8,1> q;

        //Methods
        public:

        DQ( const Matrix<double,8,1>& v);

        DQ( const Matrix<double,4,1>& v);

        DQ( const double& q0=0.0, const double& q1=0.0, const double& q2=0.0, const double& q3=0.0, const double& q4=0.0, const double& q5=0.0, const double& q6=0.0, const double& q7=0.0);

        static DQ unitDQ( const double& rot_angle, const int& x_axis, const int& y_axis, const int& z_axis, const double& x_trans, const double& y_trans, const double& z_trans);

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

        DQ rot_axis() const;
        inline DQ rotation_axis() const{return rot_axis();};

        double rot_angle() const;
        inline DQ rotation_angle() const{return rot_angle();};

        DQ log() const;

        DQ exp() const;

        DQ tplus() const;
        inline DQ T() const{return tplus();};

        DQ pinv() const;

        Matrix4d Hplus4() const;
        inline Matrix4d hamiplus4() const{return Hplus4();};

        Matrix4d Hminus4() const;
        inline Matrix4d haminus4() const{return Hminus4();};

        Matrix<double,8,8> Hplus8() const;
        inline Matrix<double,8,8> hamiplus8() const{return Hplus8();};
        
        Matrix<double,8,8> Hminus8() const;
        inline Matrix<double,8,8> haminus8() const{return Hminus8();}; 

        Vector4d vec4() const;

        Matrix<double,8,1> vec8() const;

        Matrix<double,8,8> generalizedJacobian() const;
            //The MATLAB syntax, kept for legacy reasons.
            Matrix<double,8,8> jacobG() const;

        // Operators overload functions
      public:


      //Operator (+) Overload

      friend DQ operator+(DQ dq1, DQ dq2);


      friend DQ operator+(DQ dq, int scalar);

      friend DQ operator+(int scalar, DQ dq);


      friend DQ operator+(DQ dq, float scalar);

      friend DQ operator+(float scalar, DQ dq);


      friend DQ operator+(DQ dq, double scalar);

      friend DQ operator+(double scalar, DQ dq);

      //Operator (-) Overload

      friend DQ operator-(DQ dq1, DQ dq2);


      friend DQ operator-(DQ dq, int scalar);

      friend DQ operator-(int scalar, DQ dq);


      friend DQ operator-(DQ dq, float scalar);

      friend DQ operator-(float scalar, DQ dq);


      friend DQ operator-(DQ dq, double scalar);

      friend DQ operator-(double scalar, DQ dq);


        //Operator (*) Overload

      friend DQ operator*(DQ dq1, DQ dq2);


      friend DQ operator*(DQ dq, int scalar);

      friend DQ operator*(int scalar, DQ dq);


      friend DQ operator*(DQ dq, float scalar);

      friend DQ operator*(float scalar, DQ dq);


      friend DQ operator*(DQ dq, double scalar);

      friend DQ operator*(double scalar, DQ dq);


        //Operator (==) Overload

      bool operator==(DQ dq2);


      friend bool operator==(DQ dq, int scalar);

      friend bool operator==(int scalar, DQ dq);


      friend bool operator==(DQ dq, float scalar);

      friend bool operator==(float scalar, DQ dq);


      friend bool operator==(DQ dq, double scalar);

      friend bool operator==(double scalar, DQ dq);


        //Operator (!=) Overload

      bool operator!=(DQ dq2);


      friend bool operator!=(DQ dq, int scalar);

      friend bool operator!=(int scalar, DQ dq);


      friend bool operator!=(DQ dq, float scalar);

      friend bool operator!=(float scalar, DQ dq);


      friend bool operator!=(DQ dq, double scalar);

      friend bool operator!=(double scalar, DQ dq);


      friend DQ operator^(DQ dq, double m);

      //Operator (<<) Overload

      friend std::ostream& operator<<(std::ostream& os, DQ dq);


    };//DQ Class END

    /************************************************************************
    ************** DUAL QUATERNION CONSTANTS AND OPERATORS ******************
    ************************************************************************/
    //Note that these are part of DQRobotics namespace when you include this
    //header file.

    //Constants
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

    DQ rot_axis(const DQ& dq);
    inline DQ rotation_axis(const DQ& dq){return rot_axis(dq);};

    double rot_angle(const DQ& dq);
    inline DQ rotation_angle(const DQ& dq){return rot_angle(dq);};

    DQ log(const DQ& dq);

    DQ exp(const DQ& dq);

    DQ tplus(const DQ& dq);
    inline DQ T(const DQ& dq){return tplus(dq);};

    DQ pinv(const DQ& dq);

    DQ dec_mult(const DQ& dq1, const DQ& dq2);

    Matrix4d Hplus4(const DQ& dq);
    inline Matrix4d hamiplus4(const DQ& dq){return Hplus4(dq);};

    Matrix4d Hminus4(const DQ& dq);
    inline Matrix4d haminus4(const DQ& dq){return Hminus4(dq);};

    Matrix<double,8,8> Hplus8(const DQ& dq);
    inline Matrix<double,8,8> hamiplus8(const DQ& dq){return Hplus8(dq);};

    Matrix<double,8,8> Hminus8(const DQ& dq);
    inline Matrix<double,8,8> haminus8(const DQ& dq){return Hminus8(dq);};

    Vector4d vec4(const DQ& dq);

    Matrix<double,8,1> vec8(const DQ& dq);

    Matrix<double,8,8> generalizedJacobian(const DQ& x_E);
        //The MATLAB syntax, kept for legacy reasons.
        Matrix<double,8,8> jacobG(const DQ& x_E);


}//Namespace DQRobotics

#endif // DQ_H
