cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_BUILD_TYPE Debug)
project(NAI_PFN VERSION 0.1.0)

include(CTest)
enable_testing()

set(NAI_PFN_SOURCES main.cpp)
add_executable(NAI_PFN ${NAI_PFN_SOURCES})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)