/**
* This class DQ_kinematics represents a kinematic model of a robotic system using dual quaternions concept.
*
* TODO: REDEFINE THIS DEFINITION OF THE CLASS
* In the class definition are declared different constructors for the Dual Quaternion Kinematics object, the public methods which
* can be called by the object and also auxiliar functions and variables to intermediate the operations of the public methods.
* Some methods return a constant Dual Quaternion object, some return a constant boost vector class and some return a constant boost
* matrix class. But, all of then depends of the object caller. For displaying the results of methods, the DISPLAY and MATRIX functions
* of DQ class (also provided with this class) can be used.
* \author Mateus Rodrigues Martins (martinsrmateus@gmail.com)
* \since 11/2012
***********************************************************
*              REVISION HISTORY
***********************************************************
* 2013/11/14 Murilo Marques Marinho (murilomarinho@lara.unb.br)
             - Removed const qualifiers that were only causing
               warnings.

* 2013/04/15 Murilo Marques Marinho (murilomarinho@lara.unb.br)
             - Created setDummy() method in class, for changing
               the 'dummy' status of a joint during runtime.

* 2013/02/07 Murilo Marques Marinho (murilomarinho@lara.unb.br)
             - Removed static functions and created DQRobotics
               namespace.
             - All jacobians have a more human-readable name now
               but the old names were kept for legacy reasons:
                - jacobian() -> analyticalJacobian()
                - jacobp()   -> translationJacobian()
                - jacobd()   -> distanceJacobian()
             - static methods returning constant matrix objects
               completely removed from DQ_kinematics class.

* 2013/31/01 Murilo Marques Marinho (murilomarinho@lara.unb.br)
             - Changed Library to Use Eigen.
***********************************************************
* \version 1.2
*/


#include <iostream>
#include <math.h> //library for math functions
#include "DQ.h"
#include <stdarg.h> //library to use string
#include <Eigen/Dense>

#ifndef DQ_KINEMATICS_H
#define DQ_KINEMATICS_H

using namespace Eigen;


namespace DQ_robotics
{



    class DQ_kinematics{

        // private attributtes
        private:

        //Para uso nas funções Jacobian...
        MatrixXd dq_kin;
        DQ curr_base;
        DQ curr_effector;

        std::string aux_type;


        // public methods
        public:
        // Class constructors: Creates a Dual Quaternion as a DQ object.

        DQ_kinematics(MatrixXd A);

        DQ_kinematics(MatrixXd A, std::string type);

        DQ_kinematics(){};

        ~DQ_kinematics();


        /*
        * Public constant methods: Can be called by DQ_kinematics objects.
        * To use these methods, type: 'dq_kinematics_object.method_name();' where 'method_name' is the name of one of the methods below.
        * Or in another way type: 'DQ_kinematics::method_name(dq_kinematics_object);' that works well too.
        * For displaying the results of methods, the DISPLAY and MATRIX functions of DQ class can be used
        */

        MatrixXd getDHMatrix();

        int links();

        VectorXd theta();

        VectorXd d();

        VectorXd a();

        VectorXd alpha();

        VectorXd dummy();
        void setDummy( VectorXd dummy_vector);

        int n_dummy();

        std::string convention();

        DQ base();

        DQ effector();

        DQ set_base(DQ new_base);

        DQ set_effector(DQ new_effector);

        DQ raw_fkm(VectorXd theta_vec);
        DQ raw_fkm(VectorXd theta_vec, int ith);

        DQ fkm(VectorXd theta_vec);
        DQ fkm(VectorXd theta_vec, int ith);

        DQ dh2dq(double theta_ang, int link_i);

        DQ get_z(VectorXd q);

        MatrixXd analyticalJacobian(VectorXd theta_vec);
            MatrixXd jacobian(VectorXd theta_vec); //The MATLAB syntax, kept for legacy reasons.

    };


    Matrix<double,8,8> C8();

    Matrix<double,4,4> C4();

    int links(DQ_kinematics param_dq_kin);

    VectorXd theta(DQ_kinematics param_dq_kin);

    VectorXd d(DQ_kinematics param_dq_kin);

    VectorXd a(DQ_kinematics param_dq_kin);

    VectorXd alpha(DQ_kinematics param_dq_kin);

    VectorXd dummy(DQ_kinematics param_dq_kin);

    int n_dummy(DQ_kinematics param_dq_kin);

    std::string convention(DQ_kinematics param_dq_kin);

    DQ base(DQ_kinematics param_dq_kin);

    DQ effector(DQ_kinematics param_dq_kin);

    DQ set_base(DQ_kinematics param_dq_kin, DQ new_base);

    DQ set_effector(DQ_kinematics param_dq_kin, DQ new_effector);

    DQ raw_fkm(DQ_kinematics param_dq_kin, VectorXd theta_vec);
    DQ raw_fkm(DQ_kinematics param_dq_kin, VectorXd theta_vec, int ith);

    DQ dh2dq(DQ_kinematics param_dq_kin, double theta_ang, int link_i);

    DQ get_z(DQ_kinematics param_dq_kin, VectorXd q);
   
    MatrixXd analyticalJacobian(DQ_kinematics param_dq_kin, VectorXd theta_vec);
        MatrixXd jacobian(DQ_kinematics param_dq_kin, VectorXd theta_vec); //The MATLAB syntax, kept for legacy reasons.

	MatrixXd rotationJacobian(MatrixXd analytical_jacobian);

    MatrixXd translationJacobian(MatrixXd analytical_jacobian, Matrix<double,8,1> x);
        MatrixXd  jacobp(MatrixXd analytical_jacobian, Matrix<double,8,1> x); //The MATLAB syntax, kept for legacy reasons.
    

    MatrixXd distanceJacobian(DQ_kinematics param_dq_kin, MatrixXd param_jacobian, Matrix<double,8,1> x);
        MatrixXd  jacobd(DQ_kinematics param_dq_kin, MatrixXd param_jacobian, Matrix<double,8,1> x); //The MATLAB syntax, kept for legacy reasons.

    MatrixXd pseudoInverse(MatrixXd matrix);
    

}//Namespace DQRobotics

#endif // DQ_KINEMATICS_H_INCLUDED
