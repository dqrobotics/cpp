#include <iostream>
#include<math.h> //library for math functions
#include "DQ.h"
#include<stdarg.h> //library to use string
#include <boost/numeric/ublas/vector.hpp> //header for boost ublas vector declarations
#include <boost/numeric/ublas/matrix.hpp> //header for boost ublas matrix declarations
#include <boost/numeric/ublas/io.hpp>

#ifndef DQ_KINEMATICS_H
#define DQ_KINEMATICS_H

using namespace boost::numeric::ublas;

/**
* This class DQ_kinematics represents a kinematic model of a robotic system using dual quaternions concept.
*
* TODO: REDEFINE THIS DEFINITION OF THE CLASS
* In the class definition are declared different constructors for the Dual Quaternion object, the public methods which can be called
* by the object, the operators overload functions e also auxiliar functions and variables to intermediate the operations of the
* public methods. Most of the methods returns a constant Dual Quaternion object, which depends of the object caller such as primary
* and dual parts or not, being the same for any caller such as the imaginary parts. Some methods returns a constant boost matrix class
* object which depends of object caller too. And there is a method for display in the console, the DQ object caller.
* \author Mateus Rodrigues Martins (martinsrmateus@gmail.com)
* \since 11/2012
* \version 1.0
*/
class DQ_kinematics{

    // private attributtes
    private:

    //Para uso nas funções Jacobian...
    matrix <double> dq_kin;
    DQ curr_base;
    DQ curr_effector;
    // TODO: IMPLEMENT THESE VARIABLES REGARDING ROBOTICS TOOLBOX
    // Properties for interfacing with Robotics Toolbox
    // name;
    // robot_RT;
    std::string aux_type;


    // public methods
    public:
    // Class constructors: Creates a Dual Quaternion as a DQ object.

    DQ_kinematics(matrix <double> A);

    DQ_kinematics(matrix <double> A, std::string type);

    ~DQ_kinematics();


    /*
    * Public constant methods: Can be called by DQ_kinematics objects.
    * To use these methods, type: 'dq_object.method_name();' where 'method_name' is the name of one of the methods below.
    * Or in another way type: 'DQ::method_name(dq_object);' that works well too.
    * These ways of calling function can't be applied to display() method that uses a macro called DISPLAY.
    */

    int const links();
    static int const links(DQ_kinematics param_dq_kin);

    vector <double> const theta();
    static vector <double> const theta(DQ_kinematics param_dq_kin);

    vector <double> const d();
    static vector <double> const d(DQ_kinematics param_dq_kin);

    vector <double> const a();
    static vector <double> const a(DQ_kinematics param_dq_kin);

    vector <double> const alpha();
    static vector <double> const alpha(DQ_kinematics param_dq_kin);

    vector <double> const dummy();
    static vector <double> const dummy(DQ_kinematics param_dq_kin);

    int const n_dummy();
    static int const n_dummy(DQ_kinematics param_dq_kin);

    std::string const convention();
    static std::string const convention(DQ_kinematics param_dq_kin);

    DQ const base();
    static DQ const base(DQ_kinematics param_dq_kin);

    DQ const effector();
    static DQ const effector(DQ_kinematics param_dq_kin);

    static matrix <double> const C8();
    static matrix <double> const C8(DQ_kinematics param_dq_kin);

    static matrix <double> const C4();
    static matrix <double> const C4(DQ_kinematics param_dq_kin);

    DQ const set_base(DQ new_base);
    static DQ const set_base(DQ_kinematics param_dq_kin, DQ new_base);

    DQ const set_effector(DQ new_effector);
    static DQ const set_effector(DQ_kinematics param_dq_kin, DQ new_effector);

    DQ const raw_fkm(vector <double> theta_vec);
    static DQ const raw_fkm(DQ_kinematics param_dq_kin, vector <double> theta_vec);
    DQ const raw_fkm(vector <double> theta_vec, int ith);
    static DQ const raw_fkm(DQ_kinematics param_dq_kin, vector <double> theta_vec, int ith);

    DQ const fkm(vector <double> theta_vec);
    static DQ const fkm(DQ_kinematics param_dq_kin, vector <double> theta_vec);
    DQ const fkm(vector <double> theta_vec, int ith);
    static DQ const fkm(DQ_kinematics param_dq_kin, vector <double> theta_vec, int ith);

    DQ const dh2dq(double theta_ang, int link_i);
    static DQ const dh2dq(DQ_kinematics param_dq_kin, double theta_ang, int link_i);

    DQ const get_z(vector <double> q);
    static DQ const get_z(DQ_kinematics param_dq_kin, vector <double> q);

    matrix <double> const jacobian(vector <double> theta_vec);
    static matrix <double> const jacobian(DQ_kinematics param_dq_kin, vector <double> theta_vec);

    static matrix <double> const jacobp(matrix <double> param_jacobian, vector <double> x);
    static matrix <double> const jacobp(DQ_kinematics param_dq_kin, matrix <double> param_jacobian, vector <double> x);

    static matrix <double> const jacobd(matrix <double> param_jacobian, vector <double> x);
    static matrix <double> const jacobd(DQ_kinematics param_dq_kin, matrix <double> param_jacobian, vector <double> x);
};

#endif // DQ_KINEMATICS_H_INCLUDED
