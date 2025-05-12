# Updated 2025.05 to 3.10 since compatibility with CMake < 3.10 will be removed from a future version of CMake.
CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

if(WIN32)
    set(CMAKE_TOOLCHAIN_FILE C:/vcpkg/scripts/buildsystems/vcpkg.cmake)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

PROJECT(dqrobotics)
set (CMAKE_CXX_STANDARD 11)

INCLUDE_DIRECTORIES(include)

if(UNIX AND NOT APPLE)
    FIND_PACKAGE(Eigen3 REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
    ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
endif()

if(APPLE)
    INCLUDE_DIRECTORIES(
        /usr/local/include/
        /usr/local/include/eigen3
        # Most recent versions of brew install here
        /opt/homebrew/include
        /opt/homebrew/include/eigen3)
    ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
endif()

if(WIN32)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES) 
    FIND_PACKAGE(Eigen3 CONFIG REQUIRED) 
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR}) 
endif()


################################################################
# DEFINE AND INSTALL LIBRARY AND INCLUDE FOLDER
################################################################

# As of 2025.05, Not using GLOB_RECURSE as it's explicitly discouraged.
# https://stackoverflow.com/questions/3201154/automatically-add-all-files-in-a-folder-to-a-target-using-cmake
# https://cmake.org/cmake/help/latest/command/file.html#filesystem
ADD_LIBRARY(dqrobotics SHARED 
    src/DQ.cpp

    src/internal/_dq_linesegment.cpp

    src/utils/DQ_Geometry.cpp
    src/utils/DQ_LinearAlgebra.cpp
    src/utils/DQ_Math.cpp

    src/robot_modeling/DQ_CooperativeDualTaskSpace.cpp
    src/robot_modeling/DQ_Kinematics.cpp
    src/robot_modeling/DQ_SerialManipulator.cpp
    src/robot_modeling/DQ_SerialManipulatorDH.cpp
    src/robot_modeling/DQ_SerialManipulatorMDH.cpp
    src/robot_modeling/DQ_SerialManipulatorDenso.cpp
    src/robot_modeling/DQ_MobileBase.cpp
    src/robot_modeling/DQ_HolonomicBase.cpp
    src/robot_modeling/DQ_DifferentialDriveRobot.cpp
    src/robot_modeling/DQ_WholeBody.cpp
    src/robot_modeling/DQ_SerialWholeBody.cpp

    src/robot_control/DQ_KinematicController.cpp
    src/robot_control/DQ_PseudoinverseController.cpp
    src/robot_control/DQ_NumericalFilteredPseudoInverseController.cpp
    src/robot_control/DQ_KinematicConstrainedController.cpp
    src/robot_control/DQ_QuadraticProgrammingController.cpp
    src/robot_control/DQ_ClassicQPController.cpp

    src/robots/Ax18ManipulatorRobot.cpp
    src/robots/BarrettWamArmRobot.cpp
    src/robots/ComauSmartSixRobot.cpp
    src/robots/KukaLw4Robot.cpp
    src/robots/KukaYoubotRobot.cpp
    src/robots/FrankaEmikaPandaRobot.cpp
    )

SET_TARGET_PROPERTIES(dqrobotics 
    PROPERTIES PUBLIC_HEADER
    include/dqrobotics/DQ.h
    )

INSTALL(TARGETS dqrobotics 
# https://stackoverflow.com/questions/21592361/cmake-install-is-not-installing-libraries-on-windows    
	RUNTIME DESTINATION "bin"
    LIBRARY DESTINATION "lib"
	ARCHIVE DESTINATION "lib"
    PUBLIC_HEADER DESTINATION "include/dqrobotics"
    PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ)

################################################################
# INSTALL HEADERS IN SUBFOLDERS
################################################################

INSTALL(DIRECTORY
    include/
    DESTINATION "include")

################################################################
# INSTALL SOURCE FILES (So that the debugger can find them)
################################################################

INSTALL(DIRECTORY
    src/
    DESTINATION "src/dqrobotics")
