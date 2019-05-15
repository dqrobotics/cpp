CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
PROJECT(dqrobotics)
set (CMAKE_CXX_STANDARD 11)

#ADD_DEFINITIONS(-g -O2 -Wall)
ADD_DEFINITIONS(-D_USE_MATH_DEFINES) #Need this to find M_PI on Windows

FIND_PACKAGE(Eigen3 REQUIRED) 
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR}) #Writing this way is more OS agnostic

INCLUDE_DIRECTORIES(include)

################################################################
# DEFINE AND INSTALL LIBRARY AND INCLUDE FOLDER
################################################################

ADD_LIBRARY(dqrobotics SHARED 
    src/DQ.cpp

    src/utils/DQ_Geometry.cpp
    src/utils/DQ_LinearAlgebra.cpp

    src/robot_modeling/DQ_CooperativeDualTaskSpace.cpp
    src/robot_modeling/DQ_Kinematics.cpp
    src/robot_modeling/DQ_SerialManipulator.cpp
    src/robot_modeling/DQ_MobileBase.cpp
    src/robot_modeling/DQ_HolonomicBase.cpp
    src/robot_modeling/DQ_DifferentialDriveRobot.cpp
    src/robot_modeling/DQ_WholeBody.cpp

    src/robot_control/DQ_KinematicController.cpp
    src/robot_control/DQ_TaskSpacePseudoInverseController.cpp

    src/robots/Ax18ManipulatorRobot.cpp
    src/robots/BarrettWamArmRobot.cpp
    src/robots/ComauSmartSixRobot.cpp
    src/robots/KukaLw4Robot.cpp
    )

SET_TARGET_PROPERTIES(dqrobotics 
    PROPERTIES PUBLIC_HEADER
    include/dqrobotics/DQ.h
    )

INSTALL(TARGETS dqrobotics 
    LIBRARY DESTINATION "lib"
    PUBLIC_HEADER DESTINATION "include/dqrobotics"
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

################################################################
# INSTALL HEADERS IN SUBFOLDERS
################################################################

# utils headers
INSTALL(FILES
    include/dqrobotics/utils/DQ_Math.h
    include/dqrobotics/utils/DQ_Geometry.h
    include/dqrobotics/utils/DQ_LinearAlgebra.h
    include/dqrobotics/utils/DQ_Constants.h
    DESTINATION "include/dqrobotics/utils")

# robot_modeling headers
INSTALL(FILES
    include/dqrobotics/robot_modeling/DQ_CooperativeDualTaskSpace.h
    include/dqrobotics/robot_modeling/DQ_Kinematics.h
    include/dqrobotics/robot_modeling/DQ_SerialManipulator.h
    include/dqrobotics/robot_modeling/DQ_MobileBase.h
    include/dqrobotics/robot_modeling/DQ_HolonomicBase.h
    include/dqrobotics/robot_modeling/DQ_DifferentialDriveRobot.h
    include/dqrobotics/robot_modeling/DQ_WholeBody.h
    DESTINATION "include/dqrobotics/robot_modeling")

INSTALL(FILES
    include/dqrobotics/robot_control/DQ_KinematicController.h
    include/dqrobotics/robot_control/DQ_TaskSpacePseudoInverseController.h
    DESTINATION "include/dqrobotics/robot_control")

# robots headers
INSTALL(FILES
    include/dqrobotics/robots/Ax18ManipulatorRobot.h
    include/dqrobotics/robots/BarrettWamArmRobot.h
    include/dqrobotics/robots/ComauSmartSixRobot.h
    include/dqrobotics/robots/KukaLw4Robot.h
    DESTINATION "include/dqrobotics/robots")

################################################################
# INSTALL SOURCE FILES (So that the debugger can find them)
################################################################

# base folder
INSTALL(FILES 
    src/DQ.cpp
    DESTINATION "src/dqrobotics")

# utils folder
INSTALL(FILES
    src/utils/DQ_Geometry.cpp
    src/utils/DQ_LinearAlgebra.cpp
    DESTINATION "src/dqrobotics/utils")

# robot_control folder
INSTALL(FILES
    src/robot_control/DQ_KinematicController.cpp
    src/robot_control/DQ_TaskSpacePseudoInverseController.cpp
    DESTINATION "src/dqrobotics/robot_control")

# robot_modeling folder
INSTALL(FILES 
    src/robot_modeling/DQ_SerialManipulator.cpp
    src/robot_modeling/DQ_CooperativeDualTaskSpace.cpp
    src/robot_modeling/DQ_Kinematics.cpp
    src/robot_modeling/DQ_MobileBase.cpp
    src/robot_modeling/DQ_DifferentialDriveRobot.cpp
    src/robot_modeling/DQ_WholeBody.cpp
    DESTINATION "src/dqrobotics/robot_modeling")

# robots folder
INSTALL(FILES
    src/robots/Ax18ManipulatorRobot.cpp
    src/robots/BarrettWamArmRobot.cpp
    src/robots/ComauSmartSixRobot.cpp
    src/robots/KukaLw4Robot.cpp
    DESTINATION "src/dqrobotics/robots")
