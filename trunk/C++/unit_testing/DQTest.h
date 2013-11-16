/**
Unit tests header file for testing both DQ.cpp and DQ_kinematics.cpp

\author Murilo Marques Marinho
\since 01/2013

*/


#ifndef DQTEST_H
#define DQTEST_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <Eigen/Dense>
#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../controllers/HInfinityRobustController.h"
#include "../controllers/DampedNumericalFilteredController.h"
#include "../controllers/TranslationFirstPoseController.h"
#include <iostream>

using namespace Eigen;
using namespace DQ_robotics;

class DQTest : public CppUnit::TestFixture
{

	CPPUNIT_TEST_SUITE (DQTest);
  CPPUNIT_TEST (constructorTest);
  CPPUNIT_TEST (displayTest);
	CPPUNIT_TEST (sumTest);
	CPPUNIT_TEST (subtractTest);
  CPPUNIT_TEST (copyTest);
	CPPUNIT_TEST (Hplus4Test);
	CPPUNIT_TEST (Hminus4Test);
	CPPUNIT_TEST (kinematicsTest);
	CPPUNIT_TEST_SUITE_END ();



private:

public:
	void setUp();
	void tearDown();

protected:
  void constructorTest();

  void displayTest();

	void sumTest();
	void subtractTest();
  void copyTest();
	void Hplus4Test();
	void Hminus4Test();


	void kinematicsTest();



};

#endif
