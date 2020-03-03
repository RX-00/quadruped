CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

# set the project name
PROJECT(quadruped_test_srvs)

# specify the C++ standard
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED True)

# add RapaPololuMaestro library
#ADD_SUBDIRECTORY(lib_RPM)

# add the executable
ADD_EXECUTABLE(quadruped_test_srvs RPM_test.cpp)

target_link_libraries(quadruped_test_srvs PUBLIC lib_RPM)
