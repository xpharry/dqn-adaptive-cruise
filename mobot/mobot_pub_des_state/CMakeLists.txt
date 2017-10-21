cmake_minimum_required(VERSION 2.8.3)
project(mobot_pub_des_state)

if (CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

find_package(catkin_simple REQUIRED)

find_package(Boost REQUIRED COMPONENTS system thread)
catkin_simple()

cs_add_executable(mobot_pub_des_state src/pub_des_state_main.cpp src/pub_des_state.cpp)
cs_add_executable(open_loop_controller src/open_loop_controller.cpp)
cs_add_executable(pub_des_state_path_client src/pub_des_state_path_client.cpp)
cs_add_executable(pub_des_state_flush_path_client src/pub_des_state_flush_path_client.cpp)
cs_add_executable(pub_des_state_path_replanning_client src/pub_des_state_path_replanning_client.cpp)
cs_add_executable(estop src/estop.cpp)

cs_install()
cs_export()
    
