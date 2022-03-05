cmake_minimum_required(VERSION 2.8.3)
project(pouring_control_pkg)



find_package(catkin_simple REQUIRED)

catkin_python_setup()

catkin_simple()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(Eigen3 REQUIRED)
#find_package(Boost REQUIRED COMPONENTS program_options)
# include_directories(${Boost_INCLUDE_DIRS})


include_directories(${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

cs_add_executable( meas_state_node_vision_and_scale_kf  src/meas_state_node_vision_and_scale_kf.cpp)

cs_install()
cs_export()






