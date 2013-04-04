#
# Run "make test" to create unit testing executable and run tests.
# 31/01/2013

test:
	g++ -Wall DQTest.cpp dqtest_main.cpp ../DQ.cpp ../DQ_kinematics.cpp ../controllers/HInfinityRobustController.cpp ../controllers/DampedNumericalFilteredController.cpp ../controllers/TranslationFirstPoseController.cpp -lcppunit -o test
	./test
runtest:
	./test
cleanTest:
	rm test
clean:
	rm test

