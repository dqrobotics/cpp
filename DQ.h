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
* 01/31/2013 - Murilo Marques Marinho (murilomarinho@lara.unb.br)
             - Changed Library to Use Eigen.
***********************************************************
* 02/07/2013 - Murilo Marques Marinho (murilomarinho@lara.unb.br)
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
        DQ const P();

        DQ const D();

        DQ const Re();

        DQ const Im();

        DQ const conj();

        DQ const norm();

        DQ const inv();

        DQ const translation();

        DQ const rot_axis();

        double const rot_angle();

        DQ const log();

        DQ const exp();

        DQ const tplus();

        DQ const pinv();

        Matrix4d const Hplus4();

        Matrix4d const Hminus4();

        Matrix<double,8,8> const Hplus8();

        Matrix<double,8,8> const Hminus8();

        Vector4d const vec4();

        Matrix<double,8,1> const vec8();

        Matrix<double,8,8> const generalizedJacobian(DQ x_E);
            //The MATLAB syntax, kept for legacy reasons.
            Matrix<double,8,8> const jacobG(DQ x_E);

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

        friend std::ostream& operator<<(std::ostream& os, const DQ& dq);


    };//DQ Class END


    /************************************************************************
    ************** DUAL QUATERNION CONSTANTS AND OPERATORS ******************
    ************************************************************************/
    //Note that these are part of DQRobotics namespace when you include this
    //header file.

    //Constants
    DQ const E_ = DQ(0,0,0,0,1,0,0,0);

    DQ const i_ = DQ(0,1,0,0,0,0,0,0);

    DQ const j_ = DQ(0,0,1,0,0,0,0,0);

    DQ const k_ = DQ(0,0,0,1,0,0,0,0);

    double const DQ_threshold = 0.000000000001;

    //Operators
    DQ const P(DQ dq);

    DQ const D(DQ dq);

    DQ const Re(DQ dq);

    DQ const Im(DQ dq);

    DQ const conj(DQ dq);

    DQ const norm(DQ dq);

    DQ const inv(DQ dq);

    DQ const translation(DQ dq);

    DQ const rot_axis(DQ dq);

    double const rot_angle(DQ dq);

    DQ const log(DQ dq);

    DQ const exp(DQ dq);

    DQ const tplus(DQ dq);

    DQ const pinv(DQ dq);

    DQ const dec_mult(DQ dq1, DQ dq2);

    Matrix4d const Hplus4(DQ dq);

    Matrix4d const Hminus4(DQ dq);

    Matrix<double,8,8> const Hplus8(DQ dq);

    Matrix<double,8,8> const Hminus8(DQ dq);

    Vector4d const vec4(DQ dq);

    Matrix<double,8,1> const vec8(DQ dq);

    Matrix<double,8,8> const generalizedJacobian(DQ param_dq, DQ x_E);
        //The MATLAB syntax, kept for legacy reasons.
        Matrix<double,8,8> const jacobG(DQ param_dq, DQ x_E);


}//Namespace DQRobotics

#endif // DQ_H
