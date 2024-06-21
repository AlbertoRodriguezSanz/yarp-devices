cmake_minimum_required(VERSION 3.16)

project(exampleGripperGrasp LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP 3.7 REQUIRED COMPONENTS os dev)
endif()

add_executable(exampleGripperGrasp exampleGripperGrasp.cpp)

target_link_libraries(exampleGripperGrasp	YARP::YARP_os
                                                YARP::YARP_init
                                                YARP::YARP_dev)
