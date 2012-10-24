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

class DQ_kinematics{

    private:
    matrix <double> dq_kin;
    DQ curr_base;
//    Properties for interfacing with Robotics Toolbox
//    name;
//    robot_RT;
    private:
    // Auxiliar variables used in methods display() and build_string() for correctly display the DQ object
    std::string aux_type;

    public:

    DQ_kinematics(matrix <double> A);

    DQ_kinematics(matrix <double> A, std::string type);

    ~DQ_kinematics();

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

    static matrix <double> const C8();
    static matrix <double> const C8(DQ_kinematics param_dq_kin);

    static matrix <double> const C4();
    static matrix <double> const C4(DQ_kinematics param_dq_kin);

    DQ const set_base(DQ new_base);
    static DQ const set_base(DQ_kinematics param_dq_kin, DQ new_base);

    DQ const fkm(vector <double> theta_vec);
    static DQ const fkm(DQ_kinematics param_dq_kin, vector <double> theta_vec);
    DQ const fkm(vector <double> theta_vec, int ith);
    static DQ const fkm(DQ_kinematics param_dq_kin, vector <double> theta_vec, int ith);

    DQ const dh2dq(double theta_ang, int link_i);
    static DQ const dh2dq(DQ_kinematics param_dq_kin, double theta_ang, int link_i);

    DQ const get_p(vector <double> q);
    static DQ const get_p(DQ_kinematics param_dq_kin, vector <double> q);

    matrix <double> const jacobian(vector <double> theta_vec);
    static matrix <double> const jacobian(DQ_kinematics param_dq_kin, vector <double> theta_vec);

    static matrix <double> const jacobp(matrix <double> param_jacobian, vector <double> x);
    static matrix <double> const jacobp(DQ_kinematics param_dq_kin, matrix <double> param_jacobian, vector <double> x);

    static matrix <double> const jacobd(matrix <double> param_jacobian, vector <double> x);
    static matrix <double> const jacobd(DQ_kinematics param_dq_kin, matrix <double> param_jacobian, vector <double> x);
};

#endif // DQ_KINEMATICS_H_INCLUDED
