/**
* This class DQ_kinematics represents a kinematic model of a robotic system using dual quaternions concept.
*
* In the class definition are declared different constructors for the Dual Quaternion Kinematics object, the public methods which
* can be called by the object and also auxiliar functions and variables to intermediate the operations of the public methods.
* Some methods return a constant Dual Quaternion object, some return a constant boost vector class and some return a constant boost
* matrix class. But, all of then depends of the object caller. For displaying the results of methods, the DISPLAY and MATRIX functions
* of DQ class (also provided with this class) can be used.

***********************************************************
*              REVISION HISTORY
***********************************************************
* 2013/11/19 Murilo Marques Marinho (murilomarinho@lara.unb.br)
             - Changed constructor to use default value, instead
               of having two constructors.
             - Added range_error throw to constructor and to 
               raw_fkm().
             - Removed unecessary includes.

* 2013/11/15 Murilo Marques Marinho (murilomarinho@lara.unb.br)
             - Changed the member variable names so they make
               more sense.
               from dq_kin       to  dh_matrix_
                    aux_type         dh_matrix_convention_
                    curr_base        curr_base_
                    curr_effector    curr_effector_
             - Changed raw_fkm, fkm, jacobian functions to stop 
               making unnecessary copies of objects.
             - Added 
               - const qualifiers to specify when the arguments
                 are changed by the function.
               - const qualifiers to specify when a method
                 changes the object
               - argument references in place of copies whenever
                 possible.
             - Fixed namespace functions set_base() and
               set_effector() by passing arguments as references.
             - Changed setDummy() to set_dummy() for consistency.

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

* 2012/12/10 Mateus Rodrigues Martins (martinsrmateus@gmail.com)
             - First working version implementing MATLAB
               functionality
***********************************************************
*/

#ifndef DQ_KINEMATICS_H
#define DQ_KINEMATICS_H

#include "DQ.h"

#include <math.h>       //library for math functions
#include <stdexcept>    //For range_error
#include <Eigen/Dense>  //Library for matrix usage
#include <limits>       //Used in pseudoinverse()
#include <string>

using namespace Eigen;

namespace DQ_robotics
{



    class DQ_kinematics{

        // private attributtes
        private:

        //Para uso nas funções Jacobian...
        MatrixXd    dh_matrix_;
        std::string dh_matrix_convention_;

        DQ curr_base_;
        DQ curr_effector_;

        // public methods
        public:
        // Class constructors: Creates a Dual Quaternion as a DQ object.

        DQ_kinematics(const MatrixXd& dh_matrix, const std::string& convention = "standard" );

        DQ_kinematics(){};

        ~DQ_kinematics();


        /*
        * Public constant methods: Can be called by DQ_kinematics objects.
        * To use these methods, type: 'dq_kinematics_object.method_name();' where 'method_name' is the name of one of the methods below.
        * Or in another way type: 'DQ_kinematics::method_name(dq_kinematics_object);' that works well too.
        * For displaying the results of methods, the DISPLAY and MATRIX functions of DQ class can be used
        */

        MatrixXd getDHMatrix();

        int links() const;

        VectorXd theta() const;

        VectorXd d() const;

        VectorXd a() const;

        VectorXd alpha() const;

        VectorXd dummy() const;
        void set_dummy( const VectorXd& dummy_vector);

        int n_dummy() const;

        std::string convention() const;

        DQ base() const;

        DQ effector() const;

        DQ set_base( const DQ& new_base);

        DQ set_effector( const DQ& new_effector);

        DQ raw_fkm( const VectorXd& theta_vec) const;
        DQ raw_fkm( const VectorXd& theta_vec, const int& ith) const;

        DQ fkm( const VectorXd& theta_vec) const;
        DQ fkm( const VectorXd& theta_vec, const int& ith) const;

        DQ dh2dq( const double& theta_ang, const int& link_i) const;

        DQ get_z( const VectorXd& q) const;

        MatrixXd analyticalJacobian( const VectorXd& theta_vec) const;
            MatrixXd jacobian( const VectorXd& theta_vec) const; //The MATLAB syntax, kept for legacy reasons.

    };


    Matrix<double,8,8> C8();

    Matrix<double,4,4> C4();

    int links( const DQ_kinematics& dq_kin);

    VectorXd theta( const DQ_kinematics& dq_kin);

    VectorXd d( const DQ_kinematics& dq_kin);

    VectorXd a( const DQ_kinematics& dq_kin);

    VectorXd alpha( const DQ_kinematics& dq_kin);

    VectorXd dummy( const DQ_kinematics& dq_kin);

    int n_dummy( const DQ_kinematics& dq_kin);

    std::string convention( const DQ_kinematics& dq_kin);

    DQ base( const DQ_kinematics& dq_kin);

    DQ effector( const DQ_kinematics& dq_kin);

    DQ set_base( DQ_kinematics& dq_kin, const DQ& new_base);

    DQ set_effector( DQ_kinematics& dq_kin, const DQ& new_effector);

    DQ raw_fkm( const DQ_kinematics& dq_kin, const VectorXd& theta_vec);
    DQ raw_fkm( const DQ_kinematics& dq_kin, const VectorXd& theta_vec, const int& ith);

    DQ dh2dq( const DQ_kinematics& dq_kin, const double& theta_ang, const int& link_i);

    DQ get_z( const DQ_kinematics& dq_kin, const VectorXd& q);
   
    MatrixXd analyticalJacobian( const DQ_kinematics& dq_kin, const VectorXd& theta_vec);
        MatrixXd jacobian( const DQ_kinematics& dq_kin, const VectorXd& theta_vec); //The MATLAB syntax, kept for legacy reasons.

	  MatrixXd rotationJacobian( const MatrixXd& analytical_jacobian);

    MatrixXd translationJacobian( const MatrixXd& analytical_jacobian, const Matrix<double,8,1>& x);
        MatrixXd  jacobp( const MatrixXd& analytical_jacobian, const Matrix<double,8,1>& x); //The MATLAB syntax, kept for legacy reasons.
    

    MatrixXd distanceJacobian( const DQ_kinematics& dq_kin, const MatrixXd& param_jacobian, const Matrix<double,8,1>& x);
        MatrixXd  jacobd( const DQ_kinematics& dq_kin, const MatrixXd& param_jacobian, const Matrix<double,8,1>& x); //The MATLAB syntax, kept for legacy reasons.

    MatrixXd pseudoInverse( const MatrixXd& matrix);
    

}//Namespace DQRobotics

#endif // DQ_KINEMATICS_H_INCLUDED
