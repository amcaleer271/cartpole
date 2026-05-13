cmake_minimum_required(VERSION 3.16)

project(cartpole_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(cartpole_sim
    apps/main.cpp
)

target_include_directories(cartpole_sim PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)