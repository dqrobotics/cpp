CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(dqrobotics)
ADD_DEFINITIONS(-O2 -Wall)
ADD_LIBRARY(dqrobotics SHARED src/DQ.cpp src/DQ_kinematics.cpp)
SET_TARGET_PROPERTIES(dqrobotics PROPERTIES PUBLIC_HEADER "include/dqrobotics/DQ.h;include/dqrobotics/DQ_kinematics.h;include/dqrobotics/robot_dh/A2arm.h;include/dqrobotics/robot_dh/AX18.h;include/dqrobotics/robot_dh/Comau.h;include/dqrobotics/robot_dh/Kukka.h;include/dqrobotics/robot_dh/Schunk.h;include/dqrobotics/robot_dh/WAM4.h;include/dqrobotics/robot_dh/WAM.h")
INSTALL(TARGETS dqrobotics 
	LIBRARY DESTINATION "lib"
	PUBLIC_HEADER DESTINATION "include/dqrobotics"
        PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)
#INSTALL (
#    DIRECTORY "include"
#    DESTINATION "include"
#    FILES_MATCHING PATTERN "*.h*")

