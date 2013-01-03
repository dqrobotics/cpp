
#include <iostream>
#include<math.h> //library for math functions
#include <boost/numeric/ublas/vector.hpp> //header for boost ublas vector declarations
#include <boost/numeric/ublas/matrix.hpp> //header for boost ublas matrix declarations
#include <boost/numeric/ublas/io.hpp>

#ifndef DQ_H
#define DQ_H

using namespace boost::numeric::ublas;

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
* \version 1.0
*/
class DQ{
    // public attriubutes
	public:
	/**
	* The double Boost Vector 'q' stores the eight values of primary e dual parts of Dual Quaternion.
	*
	* To access use: 'dq_object.q(index);' where 'index' varies from 0 to 7.
	* The DQ representation is: dq_object = (q(0) + q(1)*i + q(2)*j + q(3)*k) + dual_unit_E*(q(4) + q(5)*i + q(6)*j + q(7)*k)
	* \sa DQ(), DQ(vector <double> v), DQ(const DQ& dq), DQ(double scalar), DQ(double q1,double q2,double q3,double q4,double q5,double q6,double q7,double q8);
	*/
	vector <double> q;

    // private attributes
    private:
    // Auxiliar variables used in methods display() and build_string() for correctly display the DQ object
    int has_primary_element;
    int has_dual_element;

    // public methods
    public:
    // Class constructors: Creates a Dual Quaternion as a DQ object.

    DQ();

    DQ(vector <double> v);

    DQ(double q0,double q1,double q2,double q3,double q4,double q5,double q6,double q7);

    DQ(double q0,double q1,double q2,double q3);

    DQ(double scalar);

    ~DQ();

    /*
    * Public constant methods: Can be called by DQ objects.
    * To use these methods, type: 'dq_object.method_name();' where 'method_name' is the name of one of the methods below.
    * Or in another way type: 'DQ::method_name(dq_object);' that works well too.
    * These ways of calling function can't be applied to display() method that uses a macro called DISPLAY.
    */

    static DQ const E();
    static DQ const E(DQ dq);

    static DQ const i();
    static DQ const i(DQ dq);

    static DQ const j();
    static DQ const j(DQ dq);

    static DQ const k();
    static DQ const k(DQ dq);

    DQ const P();
    static DQ const P(DQ dq);

    DQ const D();
    static DQ const D(DQ dq);

    DQ const Re();
    static DQ const Re(DQ dq);

    DQ const Im();
    static DQ const Im(DQ dq);

    DQ const conj();
    static DQ const conj(DQ dq);

    DQ const norm();
    static DQ const norm(DQ dq);

    DQ const inv();
    static DQ const inv(DQ dq);

    DQ const translation();
    static DQ const translation(DQ dq);

    DQ const rot_axis();
    static DQ const rot_axis(DQ dq);

    double const rot_angle();
    static DQ const rot_angle(DQ dq);

    DQ const log();
    static DQ const log(DQ dq);

    DQ const exp();
    static DQ const exp(DQ dq);

    DQ const tplus();
    static DQ const tplus(DQ dq);

    DQ const pinv();
    static DQ const pinv(DQ dq);

    static DQ const dec_mult(DQ dq1, DQ dq2);

    matrix <double> const Hplus4();
    static matrix <double> const Hplus4(DQ dq);

    matrix <double> const Hminus4();
    static matrix <double> const Hminus4(DQ dq);

    matrix <double> const Hplus8();
    static matrix <double> const Hplus8(DQ dq);

    matrix <double> const Hminus8();
    static matrix <double> const Hminus8(DQ dq);

    matrix <double> const vec4();
    static matrix <double> const vec4(DQ dq);

    matrix <double> const vec8();
    static matrix <double> const vec8(DQ dq);

    matrix <double> const jacobG(DQ x_E);
    static matrix <double> const jacobG(DQ param_dq, DQ x_E);

    #define DISPLAY(dq) dq.display(#dq,dq) //for calling display function more easily
    void display(char *name, DQ dq);
    #define MATRIX(mat) DQ::display(#mat, mat) //for calling display function more easily
    static void display(char *name, matrix <double> &H_or_vec);
    static void display(char *name, vector <double> &vec);

    static DQ unitDQ(double rot_angle, int x_axis,int y_axis,int z_axis, double x_trans,double y_trans, double z_trans);

    private:
    /*
    * Private methods: these are the auxiliar methods used by the public methods.
    * These methods construct and return results to be used by the public methods
    */
    std::string build_string(DQ dq, int shift);

    static double threshold();

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

	bool DQ::operator==(DQ dq2);

    friend bool operator==(DQ dq, int scalar);

	friend bool operator==(int scalar, DQ dq);

	friend bool operator==(DQ dq, float scalar);

	friend bool operator==(float scalar, DQ dq);

    friend bool operator==(DQ dq, double scalar);

	friend bool operator==(double scalar, DQ dq);

    //Operator (!=) Overload

	bool DQ::operator!=(DQ dq2);

    friend bool operator!=(DQ dq, int scalar);

	friend bool operator!=(int scalar, DQ dq);

	friend bool operator!=(DQ dq, float scalar);

	friend bool operator!=(float scalar, DQ dq);

    friend bool operator!=(DQ dq, double scalar);

	friend bool operator!=(double scalar, DQ dq);

	friend DQ operator^(DQ dq, double m);


};

#endif // DQ_H
