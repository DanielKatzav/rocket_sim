cmake_minimum_required(VERSION 3.0.2)
project(rocket_sim)
cmake_policy(SET CMP0054 NEW)


# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# Set default Debug build type (Conan requires the type to be set)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING
            "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
            FORCE)
endif (NOT CMAKE_BUILD_TYPE)



find_package(catkin REQUIRED COMPONENTS
        roscpp
        dynamic_reconfigure
        std_msgs
        sensor_msgs
        controller_manager
  	joint_state_controller
	robot_state_publisher
        )


set(CMAKE_AUTOMOC ON)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)


include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES my_gazebo_experiments
        #  CATKIN_DEPENDS other_catkin_pkg
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

#add_library(yo_sdf_joint_plugin SHARED
#        src/yo_sdf_joint_plugin.cpp
#        )
add_library(force_plugin SHARED
        src/ForcePlugin/force_plugin.cpp
        )
target_include_directories(
        force_plugin PUBLIC
        ${catkin_INCLUDE_DIRS}
)
target_link_libraries(force_plugin
        ${GAZEBO_LIBRARIES}
        ${catkin_LIBRARIES}
        )
