cmake_minimum_required(VERSION 2.8)

project(robot_egm)
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX C:/workspace/EGM/install CACHE PATH C:/workspace/EGM/install FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

set (CMAKE_CXX_STANDARD 11)

# Find default python libraries and interpreter
#find_package(PythonInterp REQUIRED)
#find_package(PythonLibs REQUIRED)

set(PYTHON_LIBRARY_DIR C:/Users/AN95540/anaconda3/libs )
set(PYTHON_INCLUDE_DIRS C:/Users/AN95540/anaconda3/include )
set(PYTHON_LIBRARIES -lpython36)

set(Boost_LIBRARY_DIR C:/workspace/boost_1_66_0/bin.v2/libs/python/build/msvc-14.0/rls/adrs-mdl-64/thrdp-wn32/thrd-mlt )
set(Boost_INCLUDE_DIR C:/workspace/boost_1_66_0 )
set(Boost_LIBRARIES -lboost_python3-vc140-mt-x64-1_66)

set(protobuf_LIBRARY_DIR C:/workspace/protobuf-3.4.1/cmake/build/Release )
set(protobuf_INCLUDE_DIR C:/workspace/protobuf-3.4.1/cmake/build/include )
set(protobuf_LIBRARIES -llibprotobuf)

set(CMAKE_INSTALL_PREFIX C:/workspace/EGM/install)



include_directories(${Boost_INCLUDE_DIR} ${PYTHON_INCLUDE_DIRS} ${protobuf_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR} ${protobuf_LIBRARY_DIR} ${PYTHON_LIBRARY_DIR})

file(GLOB SOURCES
    "robot_egm.cpp"
    "egm.pb.cc"
)
message("Source is: ${SOURCES}")
add_library(robot_egm SHARED ${SOURCES})
include_directories(${CMAKE_SOURCE_DIR})

target_link_libraries(robot_egm ${Boost_LIBRARIES} ${protobuf_LIBRARIES} ${PYTHON_LIBRARIES})

# Tweaks the name of the library to match what Python expects
set_target_properties(robot_egm PROPERTIES SUFFIX .pyd)
set_target_properties(robot_egm PROPERTIES PREFIX "")
