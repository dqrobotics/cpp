all:
	mkdir -p ../usr/lib
	g++ -Wall -fPIC ../DQ.cpp ../DQ_kinematics.cpp -o ../usr/lib/libdqrobotics.so  -shared
	mkdir -p ../usr/include/dqrobotics/
	mkdir -p ../usr/include/dqrobotics/robot_dh
	cp ../DQ.h ../usr/include/dqrobotics/DQ.h
	cp ../DQ_kinematics.h ../usr/include/dqrobotics/DQ_kinematics.h
	cp ../robot_dh/* ../usr/include/dqrobotics/robot_dh/


