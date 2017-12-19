cmake_minimum_required(VERSION 2.8.3)
project(traj_builder)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_library(traj_builder src/traj_builder.cpp)   

cs_add_executable(traj_builder_example_main src/traj_builder_example_main.cpp)

target_link_libraries(traj_builder_example_main traj_builder)

cs_install()
cs_export()
    
