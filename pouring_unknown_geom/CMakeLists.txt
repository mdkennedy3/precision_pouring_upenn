cmake_minimum_required(VERSION 2.8.3)
project(pouring_unknown_geom)

find_package(catkin_simple REQUIRED)

catkin_python_setup()

catkin_simple()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

cs_add_executable(repub_angle_vol_node src/repub_angle_vol_node.cpp)
cs_add_executable(container_edge_profile_extractor_service src/container_edge_profile_extractor_service.cpp)

cs_add_executable(libgptest src/libgptest.cpp)
cs_add_executable(model_selection_node_new_GP_enabled src/model_selection_node_new_GP_enabled.cpp)

cs_install()

#cs_install_scripts(scripts/my_script.py)

cs_export()





