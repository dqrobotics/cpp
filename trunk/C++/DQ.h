/**
* This class DQ represents a Dual Quaternion.
*
* In the class definition are declared different constructors for the Dual Quaternion object, the public methods which can be called
* by the object, the operators overload functions and also auxiliar functions and variables to intermediate the operations of the
* public methods. Most of the methods returns a constant Dual Quaternion object, which depends of the object caller such as primary
* and dual parts or not, being the same for any caller such as the imaginary parts. Some methods returns a constant boost matrix class
* object which depends of object caller too. And there is a method for display in the console, the DQ object caller.
* \author Mateus Rodrigues Martins (martinsrmateus@gmail.com)
* \since 07/2012
***********************************************************
*              REVISION HISTORY
***********************************************************
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
***********************************************************
* \version 1.2
*/

#ifndef DQ_H
#define DQ_H


#include <iostream>
#include <math.h>
#include <Eigen/Dense>
using namespace Eigen;



namespace DQ_robotics{


    class DQ{

        //Atributes
	    public:

	    Matrix<double,8,1> q;

        //Methods
        public:

        //Constructors and Destructor
        DQ();

        DQ(Matrix<double,8,1> v);

        DQ(Matrix<double,4,1> v);

        DQ(double q0,double q1,double q2,double q3,double q4,double q5,double q6,double q7);

        DQ(double q0,double q1,double q2,double q3);

        DQ(double scalar);

        static DQ unitDQ(double rot_angle, int x_axis,int y_axis,int z_axis, double x_trans,double y_trans, double z_trans);

        ~DQ();

        //Methods
        DQ P();

        DQ D();

        DQ Re();

        DQ Im();

        DQ conj();

        DQ norm();

        DQ inv();

        DQ translation();

        DQ rot_axis();

        double rot_angle();

        DQ log();

        DQ exp();

        DQ tplus();

        DQ pinv();

        Matrix4d Hplus4();

        Matrix4d Hminus4();

        Matrix<double,8,8> Hplus8();

        Matrix<double,8,8> Hminus8();

        Vector4d vec4();

        Matrix<double,8,1> vec8();

        Matrix<double,8,8> generalizedJacobian(DQ x_E);
            //The MATLAB syntax, kept for legacy reasons.
            Matrix<double,8,8> jacobG(DQ x_E);

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

    const double DQ_threshold = 0.000000000001;

    //Operators
    DQ P(DQ dq);

    DQ D(DQ dq);

    DQ Re(DQ dq);

    DQ Im(DQ dq);

    DQ conj(DQ dq);

    DQ norm(DQ dq);

    DQ inv(DQ dq);

    DQ translation(DQ dq);

    DQ rot_axis(DQ dq);

    double rot_angle(DQ dq);

    DQ log(DQ dq);

    DQ exp(DQ dq);

    DQ tplus(DQ dq);

    DQ pinv(DQ dq);

    DQ dec_mult(DQ dq1, DQ dq2);

    Matrix4d Hplus4(DQ dq);

    Matrix4d Hminus4(DQ dq);

    Matrix<double,8,8> Hplus8(DQ dq);

    Matrix<double,8,8> Hminus8(DQ dq);

    Vector4d vec4(DQ dq);

    Matrix<double,8,1> vec8(DQ dq);

    Matrix<double,8,8> generalizedJacobian(DQ param_dq, DQ x_E);
        //The MATLAB syntax, kept for legacy reasons.
        Matrix<double,8,8> jacobG(DQ param_dq, DQ x_E);


}//Namespace DQRobotics

#endif // DQ_H
