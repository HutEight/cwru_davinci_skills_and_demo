cmake_minimum_required(VERSION 2.8.3)
project(cwru_davinci_skills_and_demo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


find_package(Eigen3)


find_package(catkin REQUIRED COMPONENTS
        actionlib
        actionlib_msgs
        roscpp
        rospy
        std_msgs
        geometry_msgs
        trajectory_msgs
        tf
        Eigen3
        cwru_davinci_kinematics
        cwru_davinci_control
        cwru_davinci_msgs
)

FIND_PACKAGE( Boost 1.58.0 REQUIRED COMPONENTS
        date_time
        filesystem
        system )

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )


find_package(yaml-cpp REQUIRED)


## Generate actions in the 'action' folder
add_action_files(
        DIRECTORY action
        FILES CwruDavinciSkillControl.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        actionlib_msgs std_msgs geometry_msgs trajectory_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cwru_davinci_skills_and_demo
  CATKIN_DEPENDS  roscpp rospy std_msgs geometry_msgs trajectory_msgs tf cwru_davinci_kinematics actionlib_msgs  cwru_davinci_control
  DEPENDS system_lib Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare C++ Libraries



add_library(DavinciSkillsCommunicationsManager src/DavinciSkillsCommunicationsManager.cpp)
target_link_libraries(DavinciSkillsCommunicationsManager
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )
add_dependencies(
        DavinciSkillsCommunicationsManager cwru_davinci_skills_and_demo_generate_messages_cpp
        )

add_library(DavinciSkillsCommunicationsActionClient src/DavinciSkillsCommunicationsActionClient.cpp)
target_link_libraries(DavinciSkillsCommunicationsActionClient
        DavinciSkillsCommunicationsManager
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )
add_dependencies(
        DavinciSkillsCommunicationsActionClient cwru_davinci_skills_and_demo_generate_messages_cpp
        )

add_library(DavinciSkillsAuxiliary src/DavinciSkillsAuxiliary.cpp)
target_link_libraries(DavinciSkillsAuxiliary
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )






## Declare C++ Executables

add_executable(cwru_davinci_skills_communications_node src/cwru_davinci_skills_communications_node.cpp)
target_link_libraries(cwru_davinci_skills_communications_node
        DavinciSkillsCommunicationsManager
        DavinciSkillsAuxiliary
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES})

add_executable(action_client_test_node src/action_client_test_node.cpp)
target_link_libraries(action_client_test_node
        DavinciSkillsCommunicationsManager
        DavinciSkillsCommunicationsActionClient
        DavinciSkillsAuxiliary
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cwru_davinci_skills_and_demo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
