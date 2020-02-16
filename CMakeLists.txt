cmake_minimum_required(VERSION 2.8)
project(PxRecognize)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories(${PROJECT_SOURCE_DIR}/include)
link_directories(${PROJECT_SOURCE_DIR}/lib)

find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization)
find_package(PCAP REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# Prevent the C++ compiler from printing
# a warning when linking to PCL:
# <command-line>:0:15: warning: missing whitespace after the macro name
# because the definition "-DDISABLE_LIBUSB-1.0" contains a dash character.
remove_definitions(-DDISABLE_LIBUSB-1.0)

add_definitions(-DHAVE_PCAP)



find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})

ADD_SUBDIRECTORY(${PROJECT_SOURCE_DIR}/src)

#add_executable(PxRecognize main.cpp)
#target_link_libraries(PxRecognize ${PCL_LIBRARIES} ${Boost_LIBRARIES})
