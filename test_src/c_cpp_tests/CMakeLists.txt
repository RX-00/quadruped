CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

# set the project name
PROJECT(quadruped_test_srvo)

SET(HEADERS
   quadruped/RapaPololuMaestro/include/RPMSerialInterface.h)

SET(SOURCES
   quadruped/RapaPololuMaestro/src/RPMSerialInterface.cpp)

SET(HEADERS
   quadruped/RapaPololuMaestro/include/RPMSerialInterfacePOSIX.h)

SET(SOURCES
   quadruped/RapaPololuMaestro/src/RPMSerialInterfacePOSIX.cpp)

INCLUDE_DIRECTORIES(quadruped/RapaPololuMaestro/include)

ADD_LIBRARY(${PROJECT_NAME} STATIC ${HEADERS} ${SOURCES})

# specify the C++ standard
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED True)

# add the executable
ADD_EXECUTABLE(quadruped_test_srvo RPM_test.cxx)
