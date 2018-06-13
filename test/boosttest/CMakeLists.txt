cmake_minimum_required(VERSION 2.8)

project(hello)

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

include_directories(${Boost_INCLUDE_DIR} ${PYTHON_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIR} ${PYTHON_LIBRARY_DIR})

file(GLOB SOURCES
    "hello.cpp"
)
message("Source is: ${SOURCES}")
add_library(hello SHARED ${SOURCES})
include_directories(${CMAKE_SOURCE_DIR})

target_link_libraries(hello ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

# Tweaks the name of the library to match what Python expects
set_target_properties(hello PROPERTIES SUFFIX .pyd)
set_target_properties(hello PROPERTIES PREFIX "")
