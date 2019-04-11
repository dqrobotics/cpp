CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(dqrobotics)

#ADD_DEFINITIONS(-g -O2 -Wall)

ADD_LIBRARY(dqrobotics SHARED 
    src/DQ.cpp
    src/robot_modeling/DQ_CooperativeDualTaskSpace.cpp
    src/robot_modeling/DQ_Kinematics.cpp
    src/legacy/DQ_kinematics.cpp)

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(EIGEN3_INCLUDE_DIR)

INCLUDE_DIRECTORIES(dqrobotics include)

SET_TARGET_PROPERTIES(dqrobotics 
    PROPERTIES PUBLIC_HEADER
    "include/dqrobotics/DQ.h;
    include/dqrobotics/robot_modeling/DQ_CooperativeDualTaskSpace.h;
    include/dqrobotics/robot_modeling/DQ_Kinematics.h
    include/dqrobotics/legacy/DQ_kinematics.h;"
    )

# INCLUDE FOLDER INSTALL
INSTALL(TARGETS dqrobotics 
    LIBRARY DESTINATION "lib"
    PUBLIC_HEADER DESTINATION "include/dqrobotics"
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

# ROBOT DH HEADERS
INSTALL(FILES
    include/dqrobotics/legacy/robot_dh/A2arm.h
    include/dqrobotics/legacy/robot_dh/AX18.h
    include/dqrobotics/legacy/robot_dh/Comau.h
    include/dqrobotics/legacy/robot_dh/Kukka.h
    include/dqrobotics/legacy/robot_dh/Schunk.h
    include/dqrobotics/legacy/robot_dh/WAM4.h
    include/dqrobotics/legacy/robot_dh/WAM.h
    DESTINATION "include/dqrobotics/legacy/robot_dh")

# SOURCE FILES
# base folder
INSTALL(FILES 
    src/DQ.cpp
    DESTINATION "src/dqrobotics")
# legacy folder
INSTALL(FILES 
    src/legacy/DQ_kinematics.cpp
    DESTINATION "src/dqrobotics/legacy")
# robot_modeling folder
INSTALL(FILES 
    src/robot_modeling/DQ_CooperativeDualTaskSpace.cpp
    src/robot_modeling/DQ_Kinematics.cpp
    DESTINATION "src/dqrobotics/robot_modeling")
