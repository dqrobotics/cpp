/**
Unit tests for both DQ.cpp and DQ_kinematics.cpp

\author Murilo Marques Marinho
\since 01/2013

*/



#include "DQTest.h"

CPPUNIT_TEST_SUITE_REGISTRATION (DQTest);

void DQTest::setUp(void)
{

}

void DQTest::tearDown(void)
{

}

/*************************************************************/
/********   DQ CONSTRUCTOR TESTING             ***************/
/*************************************************************/

void DQTest::constructorTest(void)
{
	DQ dq1 = DQ(1.,2.,3.,4.,5.,6.,7.,8.);
  Matrix<double,8,1> q1_test = Matrix<double,8,1>::Zero(8,1);
  q1_test << 1.,2.,3.,4.,5.,6.,7.,8.;

  CPPUNIT_ASSERT(dq1.q == q1_test);

  DQ dq_zero = DQ();
  Matrix<double,8,1> q_zero_test = Matrix<double,8,1>::Zero(8,1);
    	
	CPPUNIT_ASSERT(dq_zero.q == q_zero_test);
}


/*************************************************************/
/********   DQ DISPLAY TESTING                 ***************/
/*************************************************************/

void DQTest::displayTest(void)
{

  std::cout << std::endl << "Dual Quaternion Display test";
	DQ dq1 = DQ(1.,2.,3.,4.,5.,6.,7.,8.);
	std::cout << std::endl << dq1 << std::endl;
}



/*************************************************************/
/********   DQ ARITHMETICS TESTING             ***************/
/*************************************************************/


void DQTest::sumTest(void)
{
	DQ dq1 = DQ(1.,2.,3.,4.,5.,6.,7.,8.);
	DQ dq2 = DQ(8.,7.,6.,5.,4.,3.,2.,1.);
	DQ dq_sum = dq1+dq2;
	DQ dq_sum_should_be = DQ(9.,9.,9.,9.,9.,9.,9.,9.);

	CPPUNIT_ASSERT(dq_sum==dq_sum_should_be);
}

void DQTest::subtractTest(void)
{
	DQ dq1 = DQ(1.,2.,3.,4.,5.,6.,7.,8.);
	DQ dq2 = DQ(2.,4.,8.,16.,32.,64.,128.,256.);
	DQ dq_sub = dq1-dq2;
	DQ dq_sub_should_be = DQ(-1.,-2.,-5.,-12.,-27.,-58.,-121.,-248.);


	CPPUNIT_ASSERT(dq_sub==dq_sub_should_be);
}

void DQTest::copyTest(void)
{
	DQ dq1 = DQ(1.,2.,3.,4.,5.,6.,7.,8.);
	DQ dq2 = DQ(8.,7.,6.,5.,4.,3.,2.,1.);
  
  dq2 = dq1;

  CPPUNIT_ASSERT(dq1==dq2);

  dq2.q(1) = 20.0;

	CPPUNIT_ASSERT(dq1!=dq2);
}


void DQTest::Hplus4Test(void)
{

	Matrix4d hplus(4,4);

	hplus << 1, -2, -3, -4,
	 	       2,  1, -4,  3,
           3,  4,  1, -2,
           4, -3,  2,  1; 

	DQ dq1 = DQ(1.,2.,3.,4.,5.,6.,7.,8.);

	Matrix4d hplus_test = dq1.Hplus4();

	CPPUNIT_ASSERT( hplus_test == hplus);
}

void DQTest::Hminus4Test(void)
{

	Matrix4d hminus(4,4);

	hminus << 1, -2, -3, -4,
	 	        2,  1,  4, -3,
            3, -4,  1,  2,
            4,  3, -2,  1;

	DQ dq1 = DQ(1.,2.,3.,4.,5.,6.,7.,8.);

	Matrix4d hminus_test = dq1.Hminus4();

	CPPUNIT_ASSERT( hminus_test == hminus);
}



void DQTest::kinematicsTest(void)
{
    const double pi2 = (3.14159/2);

  	Matrix<double,8,8> kp = Matrix<double,8,8>::Zero(8,8);
  	Matrix<double,8,1> kp_diagonal(8,1);
    kp_diagonal << 0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8;
    kp.diagonal() = kp_diagonal;

    Matrix<double,7,1> thetas;
    thetas << 0,pi2,0,0,0,0,0;
    DQ reference(1,0,0,0,0,0,0,0.652495);

   	//Robot DH
	  Matrix<double,4,7> schunk_dh(4,7);
	  schunk_dh << 0,     0,   0,     0,   0,      0,  0,
	               0.3,   0,   0.328, 0,   0.2765, 0,  0.40049,
	               0,     0,   0,     0,   0,      0,  0,
	              -pi2,   pi2,-pi2,   pi2,-pi2,    pi2,0;

	  DQ_kinematics schunk(schunk_dh,"standard");
    DQ expected_eff_pose =  DQ(0.70710725027922627373, 
                               0.0, 
                               0.70710631209293517419,
                               9.3818504632046722733e-07,
                              -6.121610518039718018e-07, 
                               0.24925143948119921067, 
                               0.0, 
                               0.46138394527094378494);

    DQ eff_pose = schunk.fkm(thetas);
    CPPUNIT_ASSERT( expected_eff_pose == eff_pose);

    HInfinityRobustController r_controller(schunk, kp);

    for(int j=0;j<2;j++)
    {   
      thetas = r_controller.getNewJointPositions(reference,thetas);
      //std::cout << std::endl << "thetas" << std::endl << thetas << std::endl;
    }   

    DampedNumericalFilteredController dn_controller(schunk, kp, 0.001, 0.1, 0.001);
    DampedNumericalFilteredController pid_dn_controller(schunk, kp, MatrixXd::Zero(8,8), MatrixXd::Zero(8,8), 0.001, 0.1, 0.001);

    for(int j=0;j<10;j++)
    {   
      thetas = dn_controller.getNewJointPositions(reference,thetas);
      //std::cout << std::endl << "thetas" << std::endl << thetas.transpose() << std::endl;
      thetas = pid_dn_controller.getNewJointPositions(reference,thetas);
      //std::cout << std::endl << "thetas" << std::endl << thetas.transpose() << std::endl;
    }   

    //Namespace and Methods Comparison
    CPPUNIT_ASSERT( schunk.links() == links(schunk));
    CPPUNIT_ASSERT( schunk.d()     == d(schunk));
    CPPUNIT_ASSERT( schunk.a()     == a(schunk));
    CPPUNIT_ASSERT( schunk.alpha() == alpha(schunk));

    //Setbase and SetEffector
    set_base(schunk,DQ(1)*(1 + 0.5*E_*(i_)));
    set_effector(schunk,DQ(1)*(1 + 0.5*E_*(j_)));
    CPPUNIT_ASSERT( schunk.base()     == DQ(1)*(1 + 0.5*E_*(i_)));
    CPPUNIT_ASSERT( schunk.effector() == DQ(1)*(1 + 0.5*E_*(j_)));

    MatrixXd cool_matrix = MatrixXd(5,5);
    cool_matrix << 8.0, 5.0, 4.0, 2.0, 1.0,
                   5.0, 8.0, 5.0, 4.0, 2.0,
                   4.0, 5.0, 8.0, 5.0, 4.0,
                   2.0, 4.0, 5.0, 8.0, 5.0,
                   1.0, 2.0, 4.0, 5.0, 8.0;

    MatrixXd pinv = pseudoInverse(cool_matrix);
    std::cout << std::endl << (cool_matrix*pinv);
    //std::cout << m << std::endl;
    //std::cout << m_pinv << std::endl;
    
	
}


