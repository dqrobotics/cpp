#
# Run "make all" to create example executables.
# 2013/02/11

all:
	g++ -Wall  schunk_hinfinitycontrol_example.cpp ../DQ.cpp ../DQ_kinematics.cpp ../controllers/HInfinityRobustController.cpp -o schunk_hinfinitycontrol_example
	g++ -Wall  wam4_translationfirstcontrol_example.cpp ../DQ.cpp ../DQ_kinematics.cpp ../controllers/TranslationFirstPoseController.cpp -o wam4_translationfirstcontrol_example
	g++ -Wall  schunk_allcontrollers_example.cpp ../DQ.cpp ../DQ_kinematics.cpp ../controllers/HInfinityRobustController.cpp ../controllers/DampedNumericalFilteredController.cpp ../controllers/DampedNumericalFilteredControllerJointLimits.cpp ../controllers/OrdinaryPoseController.cpp -o schunk_allcontrollers_example
	g++ -Wall  wam_chiaveriniwithjointlimits_example.cpp ../DQ.cpp ../DQ_kinematics.cpp ../controllers/DampedNumericalFilteredControllerJointLimits.cpp -o wam_chiaveriniwithjointlimits_example
clean:
	rm -f schunk_hinfinitycontrol_example schunk_allcontrollers_example wam4_translationfirstcontrol_example wam_chiaveriniwithjointlimits_example 

