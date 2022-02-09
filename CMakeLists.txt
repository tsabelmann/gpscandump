cmake_minimum_required(VERSION 3.16)

project(gpscandump)

set(DESCRIPTION "Application logging GPS and CAN data simultaneously")
set(VERSION "0.1.0")

set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.50 REQUIRED COMPONENTS chrono)
message(STATUS "Boost version: ${Boost_VERSION}")


add_executable(gpscandump source/main.cpp)
target_link_libraries(gpscandump PUBLIC Boost::chrono)