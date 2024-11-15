cmake_minimum_required(VERSION 2.8.3)
project(MiroSheesho)

# Use C++11 standard
add_compile_options(-std=c++11)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  roscpp
  std_msgs
  sensor_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)
find_package(Eigen3 REQUIRED)
find_package(Qt5Widgets REQUIRED)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    geometry_msgs
    roscpp 
    std_msgs 
    sensor_msgs 
    moveit_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
  DEPENDS EIGEN3
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Specify Qt5 wrapper for MOC (Meta-Object Compiler)
file(GLOB ${PROJECT_NAME}_HDRS "include/${PROJECT_NAME}/*.hpp")
qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})

# Define the executable and link libraries
add_executable(MiroSheesho src/test.cpp src/qnode.cpp)
add_dependencies(MiroSheesho ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(MiroSheesho Qt5::Widgets ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

# Install the executable
install(TARGETS MiroSheesho
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

