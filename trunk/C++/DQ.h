
#include <iostream>
#include<math.h> //library for math functions
#include <boost/numeric/ublas/vector.hpp> //header for boost ublas vector declarations
#include <boost/numeric/ublas/matrix.hpp> //header for boost ublas matrix declarations
#include <boost/numeric/ublas/io.hpp>

#ifndef DQ_H
#define DQ_H

// DQ.h
using namespace boost::numeric::ublas;

/**
* This class DQ represents a Dual Quaternion.
*
* In the class definition are declared different constructors for the Dual Quaternion object, the public methods which can be called
* by the object, the operators overload functions e also some auxiliar functions and variables to intermediate the operations of the
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
    /**
	* DQ Default constructor, dont need parameters.
	*
	* Returns a DQ object with null primary and dual part. All elements of 'q' vector are 0.
	* To create a DQ object using this, type: 'DQ dq_object();' or 'DQ dq_object;'
	*/
    DQ();

    /**
	* DQ constructor with parameter
	*
	* Returns a DQ object with the values of elements equal to the values of elements from a vector 'v' passed to constructor.
	* To create a DQ object using this, type: 'DQ dq_object(v);' where 'v' is the double boost vector.
	* if 'v' size is bigger than 8, the constructor catches only the 8 eight firs elements of 'v' and distributes on 'vector q'.
	* If 'v' size is smaller than 8, the constructor catches the elements of 'v', distributes on 'vector q' and complete the
	* rest with zeros. Remember that the first 4 elements are of primary part and the last 4 are of dual part of the quaternion.
	*/
	DQ(vector <double> v);

    DQ(double scalar);

    DQ(double q1,double q2,double q3,double q4,double q5,double q6,double q7,double q8);

    /**
	* DQ Destructor
	*
	* Deletes from memory the DQ object caller. To use this destructor, type: 'dq_object.~DQ();'. Dont need parameters.
	*/
	~DQ();

    /*
    * Public constant methods: Can be called by DQ objects.
    * To use these methods, type: 'dq_object.method_name();' where 'method_name' is the name of one of the methods below.
    * These methods actually returns the object returned by the auxiliar method 'set_method_name()'. Except the display() method.
    * This procedure is made to ensure the constancy of the object returned.
    */


	/** Returns a constant DQ object representing the dual unit epsilon.
	*
	* To use this member function, type: 'dq_object.E();'. This member function actually returns the DQ object returned by the auxiliar
	* member function 'set_E()'. This procedure is made to ensure the constancy of the object returned.
	* \return A constant DQ object.
	*/
    DQ const E();

    /**
	* Returns a constant DQ object representing the imaginary unit 'i'.
	*
	* To use this member function, type: 'dq_object.i();'. This member function actually returns the DQ object returned by the auxiliar
    * member function 'set_i()'. This procedure is made to ensure the constancy of the object returned.
    * \return A constant DQ object.
	*/
    DQ const i();

    /**
	* Returns a constant DQ object representing the imaginary unit 'j'.
	*
	* To use this member function, type: 'dq_object.j();'. This member function actually returns the DQ object returned by the auxiliar
	* member function 'set_j()'. This procedure is made to ensure the constancy of the object returned.
	* \return A constant DQ object.
	*/
    DQ const j();

    /**
	* Returns a constant DQ object representing the imaginary unit 'k'.
	*
	* To use this member function, type: 'dq_object.k();'. This member function actually returns the DQ object returned by the auxiliar
	* member function 'set_k()'. This procedure is made to ensure the constancy of the object returned.
	* \return A constant DQ object.
	*/
    DQ const k();

    /**
	* Returns a constant DQ object representing the primary part of the DQ object caller.
	*
	* To use this member function, type: 'dq_object.P();'. This member function actually returns the DQ object returned by the auxiliar
	* member function 'set_P()'. This procedure is made to ensure the constancy of the object returned.
	* \return A constant DQ object.
	*/
    DQ const P();

    /**
	* Returns a constant DQ object representing the dual part of the DQ object caller.
	*
	* To use this member function, type: 'dq_object.D();'. This member function actually returns the DQ object returned by the auxiliar
	* member function 'set_D()'. This procedure is made to ensure the constancy of the object returned.
	* \return A constant DQ object.
	*/
    DQ const D();

    /**
	* Returns a constant DQ object representing the real part of the DQ object caller.
	*
	* To use this member function, type: 'dq_object.Re();'. This member function actually returns the DQ object returned by the auxiliar
	* member function 'set_Re()'. This procedure is made to ensure the constancy of the object returned.
	* \return A constant DQ object.
	*/
    DQ const Re();

    /**
	* Returns a constant DQ object representing the imaginary part of the DQ object caller.
	*
	* To use this member function, type: 'dq_object.Im();'. This member function actually returns the DQ object returned by the auxiliar
	* member function 'set_Im()'. This procedure is made to ensure the constancy of the object returned.
	* \return A constant DQ object.
	*/
    DQ const Im();

    /**
	* Returns a constant DQ object representing the conjugate of the DQ object caller.
	*
	* To use this member function, type: 'dq_object.conj();'. This member function actually returns the DQ object returned by the
	* auxiliar member function 'set_conj()'. This procedure is made to ensure the constancy of the object returned.
	* \return A constant DQ object.
	*/
    DQ const conj();

    /**
	* Returns a constant DQ object representing the norm of the DQ object caller.
	*
	* To use this member function, type: 'dq_object.norm();'. This member function actually returns the DQ object returned by the
	* auxiliar member function 'set_norm()'. This procedure is made to ensure the constancy of the object returned.
	* \return A constant DQ object.
	*/
    DQ const norm();

    DQ const inv();

    DQ const translation();

    DQ const rotation_axis();

    DQ const log();

    DQ const exp();

    /**
	* Returns a const double Boost matrix 4x4 representing the Hamilton operator H+ of primary part of the DQ object caller.
	*
	* This operator is applied only for the primary part of DQ. This is the same as consider the DQ object a quaternion.
	* To use this member function, type: 'dq_object.Hplus4();'. This member function actually returns the Boost matrix of type double,
	* returned by the auxiliar member function 'set_Hplus4()'. This procedure is made to ensure the constancy of the object returned.
	* \return A constant boost::numeric::ublas::matrix <double> (4,4).
	*/
    matrix <double> const Hplus4();

    /**
	* Returns a constant double Boost matrix 4x4 representing the Hamilton operator H- of primary part of the DQ object caller.
	*
	* This operator is applied only for the primary part of DQ. This is the same as consider the DQ object a quaternion.
	* To use this member function, type: 'dq_object.Hminus4();'. This member function actually returns the Boost matrix of type double,
	* returned by the auxiliar member function 'set_Hminus4()'. This procedure is made to ensure the constancy of the object returned.
	* \return A constant boost::numeric::ublas::matrix <double> (4,4).
	*/
    matrix <double> const Hminus4();

    /**
	* Returns a const double Boost matrix 8x8 representing the Hamilton operator H+ of the DQ object caller.
	*
	* To use this member function, type: 'dq_object.Hplus8();'. This member function actually returns the Boost matrix of type double,
	* returned by the auxiliar member function 'set_Hplus8()'. This procedure is made to ensure the constancy of the object returned.
	* \return A constant boost::numeric::ublas::matrix <double> (8,8).
	*/
    matrix <double> const Hplus8();

    /**
	* Returns a constant double Boost matrix 8x8 representing the Hamilton operator H- of the DQ object caller.
	*
	* To use this member function, type: 'dq_object.Hminus8();'. This member function actually returns the Boost matrix of type double,
	* returned by the auxiliar member function 'set_Hminus8()'. This procedure is made to ensure the constancy of the object returned.
	* \return A constant boost::numeric::ublas::matrix <double> (8,8).
	*/
    matrix <double> const Hminus8();

    /**
	* Returns a constant double Boost matrix 4x1 representing the 'vec' operator of primary part of the DQ object caller.
	*
	* This operator is applied only for the primary part of DQ. This is the same as consider the DQ object a quaternion.
	* To use this member function, type: 'dq_object.vec4();'. This member function actually returns the Boost matrix of type double,
	* returned by the auxiliar member function 'set_vec4()'. This procedure is made to ensure the constancy of the object returned.
	* \return A constant boost::numeric::ublas::matrix <double> (4,1).
	*/
    matrix <double> const vec4();

    /**
	* Returns a const double boost matrix 8x1 representing the 'vec' operator of the DQ object caller.
	*
	* To use this member function, type: 'dq_object.vec8();'. This member function actually returns the Boost matrix of type double,
	* returned by the auxiliar member function 'set_vec8()'. This procedure is made to ensure the constancy of the object returned.
	* \return A constant boost::numeric::ublas::matrix <double> (8,1).
	*/
    matrix <double> const vec8();

    /**
	* Member function used to display the DQ object in the console.
	*
	* To use this member function, type: 'dq_object.display();'.
	* The DQ object is displayed such as: dq_object = (q(0) + q(1)*i + q(2)*j + q(3)*k) + E*(q(4) + q(5)*i + q(6)*j + q(7)*k).
	* This member function uses the auxiliar member function 'build_string()' to constructs correctly de string to be displayed.
	*/
    void display();

    static double threshold();

    private:
    /*
    * Private methods: these are the auxiliar methods used by the public methods.
    * These methods construct and return results to be returned by the public methods(or displayed in case of display function)
    */

    /**
	* Constructs and Returns a DQ object representing the dual unit epsilon.
	*
	* Creates an 8 elements Boost vector 'v' with values (0,0,0,0,1,0,0,0), in this order, and return DQ(v)
	* \return A DQ object constructed with DQ(v)
	* \sa E(), DQ(vector <double> v)
	*/
	DQ set_E();

	/**
	* Constructs and Returns a DQ object representing the imaginary unit 'i'.
	*
	* Creates an 8 elements Boost vector 'v' with values (0,1,0,0,0,0,0,0) in this order and return DQ(v).
	* \return A DQ object constructed with DQ(v)
	* \sa i(), DQ(vector <double> v)
	*/
    DQ set_i();

    /**
	* Constructs and Returns a DQ object representing the imaginary unit 'j'.
	*
	* Creates an 8 elements Boost vector 'v' with values (0,0,1,0,0,0,0,0) in this order and return DQ(v).
	* \return A DQ object constructed with DQ(v)
	* \sa j(), DQ(vector <double> v)
	*/
    DQ set_j();

    /**
	* Constructs and Returns a DQ object representing the imaginary unit 'k'.
	*
	* Creates an 8 elements Boost vector 'v' with values (0,0,0,1,0,0,0,0) in this order and return DQ(v).
	* \return A DQ object constructed with DQ(v)
	* \sa k(), DQ(vector <double> v)
	*/
    DQ set_k();

    /**
	* Constructs and Returns a DQ object representing the primary part of the DQ object caller.
	*
	* Creates an 8 elements Boost vector 'v' with values (q(0),q(1),q(2),q(3),0,0,0,0) in this order and return DQ(v).
	* \return A DQ object constructed with DQ(v)
	* \sa P(), DQ(vector <double> v)
	*/
    DQ set_P();

    /**
	* Constructs and Returns a DQ object representing the dual part of the DQ object caller.
	*
	* Creates an 8 elements Boost vector 'v' with values (q(4),q(5),q(6),q(7),0,0,0,0) in this order and return DQ(v).
	* \return A DQ object constructed with DQ(v)
	* \sa D(), DQ(vector <double> v)
	*/
    DQ set_D();

    /**
	* Constructs and Returns a DQ object representing the real part of the DQ object caller.
	*
	* Creates an 8 elements Boost vector 'v' with values (q(0),0,0,0,q(4),0,0,0) in this order and return DQ(v).
	* \return A DQ object constructed with DQ(v)
	* \sa Re(), DQ(vector <double> v)
	*/
    DQ set_Re();

    /**
	* Constructs and Returns a DQ object representing the imaginary part of the DQ object caller.
	*
	* Creates an 8 elements Boost vector 'v' with values (0,q(1),q(2),q(3),0,q(5),q(6),q(7)) in this order and return DQ(v).
	* \return A DQ object constructed with DQ(v)
	* \sa Im(), DQ(vector <double> v)
	*/
    DQ set_Im();

    /**
	* Constructs and Returns a DQ object representing the conjugate part of the DQ object caller.
	*
	* Creates an 8 elements Boost vector 'v' with values (q(0),-q(1),-q(2),-q(3),q(4),-q(5),-q(6),-q(7)) in this order and return DQ(v).
	* \return A DQ object constructed with DQ(v)
	* \sa conj(), DQ(vector <double> v)
	*/
    DQ set_conj();

    /**
	* Constructs and Returns a DQ object representing the norm of the DQ object caller.
	*
	* Creates an 8 elements Boost vector 'v' with calculated values for the norm and return DQ(v).
	* \return A DQ object constructed with DQ(v)
	* \sa norm(), DQ(vector <double> v)
	*/
    DQ set_norm();

    DQ set_inv();

    DQ set_translation();

    DQ set_rotation_axis();

    DQ set_log();

    DQ set_exp();

    /**
	* Constructs and Returns a Boost matrix representing the Hamilton operator H+ of the primary part of DQ object caller.
	*
	* Creates a 4x4 Boost matrix, fill it with values based on the elements q(0) to q(4) and return the matrix H+.
	* \return A boost::numeric::ublas::matrix <double> (4,4).
	* \sa Hplus4()
	*/
    matrix <double> set_Hplus4();

    /**
	* Constructs and Returns a Boost matrix representing the Hamilton operator H- of the primary part of DQ object caller.
	*
	* Creates a 4x4 Boost matrix, fill it with values based on the elements q(0) to q(4) and return the matrix H-.
	* \return A boost::numeric::ublas::matrix <double> (4,4).
	* \sa Hminus4()
	*/
    matrix <double> set_Hminus4();

    /**
	* Constructs and Returns a Boost matrix representing the Hamilton operator H+ of DQ object caller.
	*
	* Creates an 8x8 Boost matrix, fill it with values based on the elements q(0) to q(8) and return the matrix H+.
	* \return A boost::numeric::ublas::matrix <double> (8,8).
	* \sa Hplus8()
	*/
    matrix <double> set_Hplus8();

    /**
	* Constructs and Returns a Boost matrix representing the Hamilton operator H- of DQ object caller.
	*
	* Creates an 8x8 Boost matrix, fill it with values based on the elements q(0) to q(8) and return the matrix H-.
	* \return A boost::numeric::ublas::matrix <double> (8,8).
	* \sa Hminus4()
	*/
    matrix <double> set_Hminus8();

    /**
	* Constructs and Returns a Boost matrix representing the vec operator of the primary part of DQ object caller.
	*
	* Creates a 4x1 Boost matrix, fill it with values based on the elements q(0) to q(4) and return the column matrix vec4.
	* \return A boost::numeric::ublas::matrix <double> (4,1).
	* \sa vec4()
	*/
    matrix <double> set_vec4();

    /**
	* Constructs and Returns a Boost matrix representing the vec operator of DQ object caller.
	*
	* Creates an 8x1 Boost matrix, fill it with values based on the elements q(0) to q(8) and return the column matrix vec8.
	* \return A boost::numeric::ublas::matrix <double> (8,1).
	* \sa vec8()
	*/
    matrix <double> set_vec8();

    /**
	* Constructs and Returns a string to be displayed representing a part of the DQ object caller.
	*
	* Creates a string used by the display() member function which contains, parentesis, the imaginary units, and the elements scalar
	* values of the primary or dual part of the dual quaternion. The part being constructed is defined by a shift variable.
	* \param dq is the dual quaternion to be displayed.
	* \param shift is an integer that shifts the focus to the part which the display string is being constructed (primary or dual).
	* \return A std::string.
	* \sa display()
	*/
    std::string build_string(DQ dq, int shift);

    // Operators overload functions
	public:
	//Operator (+) Overload
	/**
	* Operator (+) overload for the sum of two DQ objects.
	*
	* This friend function realizes the sum of two DQ objects and returns the result on another DQ object which is created with default
	* constructor and have the vector 'q' modified acording to the operation.
	* \param dq1 is the first DQ object in the operation.
	* \param dq2 is the second DQ object in the operation.
	* \return A DQ object.
	* \sa DQ()
	*/
	friend DQ operator+(DQ dq1, DQ dq2);

	/**
	* Operator (+) overload for the sum of a DQ object and an integer scalar
	*
	* This friend function realizes the sum of a DQ object and an integer scalar and returns the result on another DQ object
	* which is created with default constructor and have the vector 'q' modified acording to the operation.
	* \param dq is the DQ object in the operation.
	* \param scalar is an integer scalar involved in operation.
	* \return A DQ object.
	* \sa DQ()
	*/
	friend DQ operator+(DQ dq, int scalar);

	/**
	* Operator (+) overload for the sum of an integer scalar and DQ object
	*
	* This friend function realizes the sum of an integer scalar and a DQ object and returns the result on another DQ object
	* which is created with default constructor and have the vector 'q' modified acording to the operation.
	* \param scalar is an integer scalar involved in operation.
	* \param dq is the DQ object in the operation.
	* \return A DQ object.
	* \sa DQ()
	*/
	friend DQ operator+(int scalar, DQ dq);

	/**
	* Operator (+) overload for the sum of a DQ object and a float scalar
	*
	* This friend function realizes the sum of a DQ object and a float scalar and returns the result on another DQ object
	* which is created with default constructor and have the vector 'q' modified acording to the operation.
	* \param dq is the DQ object in the operation.
	* \param scalar is a float scalar involved in operation.
	* \return A DQ object.
	* \sa DQ()
	*/
    friend DQ operator+(DQ dq, float scalar);

    /**
	* Operator (+) overload for the sum of a float scalar and DQ object
	*
	* This friend function realizes the sum of a float scalar and a DQ object and returns the result on another DQ object
	* which is created with default constructor and have the vector 'q' modified acording to the operation.
	* \param scalar is a float scalar involved in operation.
	* \param dq is the DQ object in the operation.
	* \return A DQ object.
	* \sa DQ()
	*/
	friend DQ operator+(float scalar, DQ dq);

	/**
	* Operator (+) overload for the sum of a DQ object and a double scalar
	*
	* This friend function realizes the sum of a DQ object and a double scalar and returns the result on another DQ object
	* which is created with default constructor and have the vector 'q' modified acording to the operation.
	* \param dq is the DQ object in the operation.
	* \param scalar is a double scalar involved in operation.
	* \return A DQ object.
	* \sa DQ()
	*/
    friend DQ operator+(DQ dq, double scalar);

	/**
	* Operator (+) overload for the sum of a double scalar and DQ object
	*
	* This friend function realizes the sum of a double scalar and a DQ object and returns the result on another DQ object
	* which is created with default constructor and have the vector 'q' modified acording to the operation.
	* \param scalar is a double scalar involved in operation.
	* \param dq is the DQ object in the operation.
	* \return A DQ object.
	* \sa DQ()
	*/
	friend DQ operator+(double scalar, DQ dq);

    //Operator (-) Overload
    /**
	* Operator (-) overload to subtract one DQ object of other.
	*
	* This friend function do the subtraction of a DQ object in other and returns the result on another DQ object which
	* is created with default constructor and have the vector 'q' modified acording to the operation.
	* \param dq1 is the first DQ object in the operation.
	* \param dq2 is the second DQ object in the operation.
	* \return A DQ object.
	* \sa DQ()
	*/
	friend DQ operator-(DQ dq1, DQ dq2);

    /**
	* Operator (-) overload to subtract an integer scalar of one DQ object.
	*
	* This friend function realizes the subtraction of a integer scalar in one DQ object. and returns the result on another DQ object
	* which is created with default constructor and have the vector 'q' modified acording to the operation.
	* \param dq is the DQ object in the operation.
	* \param scalar is the integer scalar involved in operation.
	* \return A DQ object.
	* \sa DQ()
	*/
	friend DQ operator-(DQ dq, int scalar);

	/**
	* Operator (-) overload to subtract a DQ object of one integer scalar.
	*
	* This friend function realizes the subtraction of a DQ object in one integer scalar. and returns the result on another DQ object
	* which is created with default constructor and have the vector 'q' modified acording to the operation.
	* \param scalar is the integer scalar involved in operation.
	* \param dq is the DQ object in the operation.
	* \return A DQ object.
	* \sa DQ()
	*/
	friend DQ operator-(int scalar, DQ dq);

    /**
	* Operator (-) overload to subtract an float scalar of one DQ object.
	*
	* This friend function realizes the subtraction of a float scalar in one DQ object. and returns the result on another DQ object
	* which is created with default constructor and have the vector 'q' modified acording to the operation.
	* \param dq is the DQ object in the operation.
	* \param scalar is the float scalar involved in operation.
	* \return A DQ object.
	* \sa DQ()
	*/
    friend DQ operator-(DQ dq, float scalar);

	/**
	* Operator (-) overload to subtract a DQ object of one float scalar.
	*
	* This friend function realizes the subtraction of a DQ object in one float scalar. and returns the result on another DQ object
	* which is created with default constructor and have the vector 'q' modified acording to the operation.
	* \param scalar is the float scalar involved in operation.
	* \param dq is the DQ object in the operation.
	* \return A DQ object.
	* \sa DQ()
	*/
	friend DQ operator-(float scalar, DQ dq);

    /**
	* Operator (-) overload to subtract an double scalar of one DQ object.
	*
	* This friend function realizes the subtraction of a double scalar in one DQ object. and returns the result on another DQ object
	* which is created with default constructor and have the vector 'q' modified acording to the operation.
	* \param dq is the DQ object in the operation.
	* \param scalar is the double scalar involved in operation.
	* \return A DQ object.
	* \sa DQ()
	*/
    friend DQ operator-(DQ dq, double scalar);

	/**
	* Operator (-) overload to subtract a DQ object of one double scalar.
	*
	* This friend function realizes the subtraction of a DQ object in one double scalar. and returns the result on another DQ object
	* which is created with default constructor and have the vector 'q' modified acording to the operation.
	* \param scalar is the double scalar involved in operation.
	* \param dq is the DQ object in the operation.
	* \return A DQ object.
	* \sa DQ()
	*/
	friend DQ operator-(double scalar, DQ dq);

    //Operator (*) Overload

    /**
	* Operator (*) overload for the standard multiplication of two DQ objects.
	*
	* This friend function do the standard multiplication of two DQ objects and returns the result on another DQ object which
	* is created with default constructor and have the vector 'q' modified acording to the operation.
	* \param dq1 is the first DQ object in the operation.
	* \param dq2 is the second DQ object in the operation.
	* \return A DQ object.
	* \sa DQ()
	*/
	friend DQ operator*(DQ dq1, DQ dq2);

	/**
	* Operator (*) overload for the multiplication of a DQ object and an integer scalar
	*
	* This friend function realizes the multiplication of a DQ object and an integer scalar and returns the result on another DQ object
	* which is created with default constructor and have the vector 'q' modified acording to the operation.
	* \param dq is the DQ object in the operation.
	* \param scalar is an integer scalar involved in operation.
	* \return A DQ object.
	* \sa DQ()
	*/
    friend DQ operator*(DQ dq, int scalar);

    /**
	* Operator (*) overload for the multiplication of an integer scalar and DQ object
	*
	* This friend function realizes the multiplication of an integer scalar and a DQ object and returns the result on another DQ object
	* which is created with default constructor and have the vector 'q' modified acording to the operation.
	* \param scalar is an integer scalar involved in operation.
	* \param dq is the DQ object in the operation.
	* \return A DQ object.
	* \sa DQ()
	*/
	friend DQ operator*(int scalar, DQ dq);

	/**
	* Operator (*) overload for the multiplication of a DQ object and a float scalar
	*
	* This friend function realizes the multiplication of a DQ object and a float scalar and returns the result on another DQ object
	* which is created with default constructor and have the vector 'q' modified acording to the operation.
	* \param dq is the DQ object in the operation.
	* \param scalar is a float scalar involved in operation.
	* \return A DQ object.
	* \sa DQ()
	*/
    friend DQ operator*(DQ dq, float scalar);

    /**
	* Operator (*) overload for the multiplication of a float scalar and DQ object
	*
	* This friend function realizes the multiplication of a float scalar and a DQ object and returns the result on another DQ object
	* which is created with default constructor and have the vector 'q' modified acording to the operation.
	* \param scalar is a float scalar involved in operation.
	* \param dq is the DQ object in the operation.
	* \return A DQ object.
	* \sa DQ()
	*/
	friend DQ operator*(float scalar, DQ dq);

	/**
	* Operator (*) overload for the multiplication of a DQ object and an double scalar
	*
	* This friend function realizes the multiplication of a DQ object and an double scalar and returns the result on another DQ object
	* which is created with default constructor and have the vector 'q' modified acording to the operation.
	* \param dq is the DQ object in the operation.
	* \param scalar is an double scalar involved in operation.
	* \return A DQ object.
	* \sa DQ()
	*/
    friend DQ operator*(DQ dq, double scalar);

    /**
	* Operator (*) overload for the multiplication of an double scalar and DQ object
	*
	* This friend function realizes the multiplication of an double scalar and a DQ object and returns the result on another DQ object
	* which is created with default constructor and have the vector 'q' modified acording to the operation.
	* \param scalar is an double scalar involved in operation.
	* \param dq is the DQ object in the operation.
	* \return A DQ object.
	* \sa DQ()
	*/
	friend DQ operator*(double scalar, DQ dq);

    //Operator (==) Overload

    /**
	* Operator (==) overload for the comparison between two DQ objects.
	*
	* This function do the comparison of two DQ objects. One is the DQ object caller, the first member in operation.
	* The result is returned as a boolean variable. True, means that both DQ objects are equal.
	* \param dq2 is the second DQ object in the operation.
	* \return A boolean variable.
	*/
	bool DQ::operator==(DQ dq2);

    /**
	* Operator (==) overload for the comparison between a DQ object and an integer scalar.
	*
	* This function do the comparison between a DQ object and an integer scalar. The scalar is transformed in a DQ object and then
	* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that both
	* DQ objects are equal and thus the DQ object is equal to the scalar.
	* \param dq is the DQ object in the operation.
	* \param scalar is an integer scalar involved in operation
	* \return A boolean variable.
	* \sa operator==(DQ dq2)
	*/
    friend bool operator==(DQ dq, int scalar);

	/**
	* Operator (==) overload for the comparison between an integer scalar and a DQ object
	*
	* This function do the comparison between a DQ object and an integer scalar. The scalar is transformed in a DQ object and then
	* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that both
	* DQ objects are equal and thus the DQ object is equal to the scalar.
	* \param scalar is an integer scalar involved in operation
	* \param dq is the DQ object in the operation.
	* \return A boolean variable.
	* \sa operator==(DQ dq2)
	*/
	friend bool operator==(int scalar, DQ dq);

    /**
	* Operator (==) overload for the comparison between a DQ object and a float scalar.
	*
	* This function do the comparison between a DQ object and a float scalar. The scalar is transformed in a DQ object and then
	* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that both
	* DQ objects are equal and thus the DQ object is equal to the scalar.
	* \param dq is the DQ object in the operation.
	* \param scalar is a float scalar involved in operation
	* \return A boolean variable.
	* \sa operator==(DQ dq2)
	*/
	friend bool operator==(DQ dq, float scalar);

	/**
	* Operator (==) overload for the comparison between a float scalar and a DQ object
	*
	* This function do the comparison between a DQ object and a float scalar. The scalar is transformed in a DQ object and then
	* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that both
	* DQ objects are equal and thus the DQ object is equal to the scalar.
	* \param scalar is a float scalar involved in operation
	* \param dq is the DQ object in the operation.
	* \return A boolean variable.
	* \sa operator==(DQ dq2)
	*/
	friend bool operator==(float scalar, DQ dq);

    /**
	* Operator (==) overload for the comparison between a DQ object and a double scalar.
	*
	* This function do the comparison between a DQ object and a double scalar. The scalar is transformed in a DQ object and then
	* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that both
	* DQ objects are equal and thus the DQ object is equal to the scalar.
	* \param dq is the DQ object in the operation.
	* \param scalar is an double scalar involved in operation
	* \return A boolean variable.
	* \sa operator==(DQ dq2)
	*/
    friend bool operator==(DQ dq, double scalar);

	/**
	* Operator (==) overload for the comparison between a double scalar and a DQ object
	*
	* This function do the comparison between a DQ object and a double scalar. The scalar is transformed in a DQ object and then
	* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that both
	* DQ objects are equal and thus the DQ object is equal to the scalar.
	* \param scalar is a double scalar involved in operation
	* \param dq is the DQ object in the operation.
	* \return A boolean variable.
	* \sa operator==(DQ dq2)
	*/
	friend bool operator==(double scalar, DQ dq);

    //Operator (!=) Overload

    /**
	* Operator (!=) overload for the comparison between two DQ objects.
	*
	* This function do the comparison of two DQ objects. One is the DQ object caller, the first member in operation.
	* The result is returned as a boolean variable. True, means that DQ objects are not equal.
	* \param dq2 is the second DQ object in the operation.
	* \return A boolean variable.
	*/
	bool DQ::operator!=(DQ dq2);

    /**
	* Operator (!=) overload for the comparison between a DQ object and an integer scalar.
	*
	* This friend function do the comparison between a DQ object and an integer scalar. The scalar is transformed in a DQ object and then
	* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that DQ objects
	* are not equal and thus the DQ object isn't equal to the scalar.
	* \param dq is the DQ object in the operation.
	* \param scalar is an integer scalar involved in operation
	* \return A boolean variable.
	* \sa operator!=(DQ dq2)
	*/
    friend bool operator!=(DQ dq, int scalar);

    /**
	* Operator (!=) overload for the comparison between an integer scalar and a DQ object
	*
	* This function do the comparison between a DQ object and an integer scalar. The scalar is transformed in a DQ object and then
	* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that DQ objects
	* are not equal and thus the DQ object isn't equal to the scalar.
	* \param scalar is an integer scalar involved in operation
	* \param dq is the DQ object in the operation.
	* \return A boolean variable.
	* \sa operator!=(DQ dq2)
	*/
	friend bool operator!=(int scalar, DQ dq);

    /**
	* Operator (!=) overload for the comparison between a DQ object and a float scalar.
	*
	* This friend function do the comparison between a DQ object and a float scalar. The scalar is transformed in a DQ object and then
	* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that DQ objects
	* are not equal and thus the DQ object isn't equal to the scalar.
	* \param dq is the DQ object in the operation.
	* \param scalar is a float scalar involved in operation
	* \return A boolean variable.
	* \sa operator!=(DQ dq2)
	*/
	friend bool operator!=(DQ dq, float scalar);

    /**
	* Operator (!=) overload for the comparison between a float scalar and a DQ object
	*
	* This function do the comparison between a DQ object and a float scalar. The scalar is transformed in a DQ object and then
	* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that DQ objects
	* are not equal and thus the DQ object isn't equal to the scalar.
	* \param scalar is a float scalar involved in operation
	* \param dq is the DQ object in the operation.
	* \return A boolean variable.
	* \sa operator!=(DQ dq2)
	*/
	friend bool operator!=(float scalar, DQ dq);

    /**
	* Operator (!=) overload for the comparison between a DQ object and a double scalar.
	*
	* This friend function do the comparison between a DQ object and a double scalar. The scalar is transformed in a DQ object and then
	* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that DQ objects
	* are not equal and thus the DQ object isn't equal to the scalar.
	* \param dq is the DQ object in the operation.
	* \param scalar is a double scalar involved in operation
	* \return A boolean variable.
	* \sa operator!=(DQ dq2)
	*/
    friend bool operator!=(DQ dq, double scalar);

    /**
	* Operator (!=) overload for the comparison between a double scalar and a DQ object
	*
	* This function do the comparison between a DQ object and a double scalar. The scalar is transformed in a DQ object and then
	* the comparison between two DQ objects is executed. The result is returned as a boolean variable. True, means that DQ objects
	* are not equal and thus the DQ object isn't equal to the scalar.
	* \param scalar is a double scalar involved in operation
	* \param dq is the DQ object in the operation.
	* \return A boolean variable.
	* \sa operator!=(DQ dq2)
	*/
	friend bool operator!=(double scalar, DQ dq);

	friend DQ operator^(DQ dq, double m);


};

#endif // DQ_H
