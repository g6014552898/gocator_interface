cmake_minimum_required(VERSION 2.8.12)

# Cmake policy
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0005 NEW) 
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Project name and the type of project
PROJECT(gocator_interface)
message(STATUS "Executig CMakeLists of project  ${PROJECT_NAME}.")

# DEBUG/RELEASE
IF (NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "RELEASE") 
ENDIF (NOT CMAKE_BUILD_TYPE)
message(STATUS "Configured to compile in ${CMAKE_BUILD_TYPE} mode.")

#Set Flags
SET(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -D_REENTRANT")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -D_REENTRANT")

## Find catkin macros and libraries
find_package(catkin REQUIRED 
    COMPONENTS 
        roscpp 
        rospy 
        dynamic_reconfigure 
        std_msgs
        sensor_msgs 
        pcl_ros
        pcl_conversions
)

# Find point cloud Library
FIND_PACKAGE(PCL 1.10 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

# Set GO_SDK include paths (That could be part of a FindGocator.cmake)
FIND_PATH(
    GOCATOR_INCLUDES
    NAMES GoSdk/GoSdk.h
    PATHS ${GO_SDK}/Gocator/GoSdk)
FIND_PATH(
    KAPI_INCLUDES
    NAMES kApi/kApi.h
    PATHS ${GO_SDK}/Platform/kApi)
INCLUDE_DIRECTORIES(${GOCATOR_INCLUDES} ${KAPI_INCLUDES})

# Set GO_SDK libs (That could be part of a FindGocator.cmake)
FIND_LIBRARY(
    GOCATOR_LIBRARIES
    NAMES GoSdk
    PATHS ${GO_SDK}/lib/linux_x64/) 
FIND_LIBRARY(
    KAPI_LIBRARIES
    NAMES kApi
    PATHS ${GO_SDK}/lib/linux_x64/) 
message(STATUS "Gocator Headers found at ${GOCATOR_INCLUDES}.")
message(STATUS "Gocator Libraries found at ${GOCATOR_LIBRARIES}.")

# add other necessary include directories
include_directories(src/ ${catkin_INCLUDE_DIRS})

# Set source files for the library
SET(SRCS_LIB 
    src/gocator_interface.cpp)
    
# Set header files for the library
SET(HDRS_LIB 
    src/gocator_interface.h)

# Set source files for the ROS node
SET(SRCS_NODE 
    src/gocator_interface_node.cpp
    src/gocator_interface_node_main.cpp
)


#dynamic reconfigure cfg file
generate_dynamic_reconfigure_options(cfg/gocator_interface_params.cfg)

## The catkin_package macro generates cmake config files for your package
catkin_package(
    CATKIN_DEPENDS 
        roscpp 
        rospy 
        dynamic_reconfigure 
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
)

#Build library 
ADD_LIBRARY(${PROJECT_NAME}_LIB SHARED ${SRCS_LIB})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_LIB ${GOCATOR_LIBRARIES} ${KAPI_LIBRARIES} ${PCL_LIBRARIES})

#Build executable
ADD_EXECUTABLE(${PROJECT_NAME} ${SRCS_NODE} ${SRCS_LIB})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${GOCATOR_LIBRARIES} ${KAPI_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)


