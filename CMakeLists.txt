cmake_minimum_required (VERSION 2.6) 
project (HELLO) 


set(youbot_driver_PACKAGE_PATH "/home/matthias/workspace/youbot_driver")

SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/external/cmake_modules")

set(VREP_DIR "." CACHE PATH "v-rep directory")

set(CMAKE_C_FLAGS "-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -Wno-write-strings")

FIND_PACKAGE(OODL_YouBot REQUIRED)

ADD_DEFINITIONS(-DYOUBOT_CONFIGURATIONS_DIR="${OODL_YOUBOT_CONFIG_DIR}/")

INCLUDE_DIRECTORIES(
        ${OODL_YOUBOT_INCLUDE_DIR}
        ${Boost_INCLUDE_DIR}
)


EXECUTE_PROCESS(COMMAND make -f v_repExtRemoteApi_Makefile
    WORKING_DIRECTORY ${VREP_DIR}/programming )

include_directories (
    include
    /usr/local/include
    /usr/include/eigen3
    ${VREP_DIR}/programming/common
    ${VREP_DIR}/programming/include
    ${VREP_DIR}/programming/remoteApi
) 

link_directories(
    ${VREP_DIR}/programming
)

add_executable (example 
    src/example.cpp 
    src/vrep/VRepRobotInterface.cpp

    ${VREP_DIR}/programming/remoteApi/extApi.c
    ${VREP_DIR}/programming/remoteApi/extApiPlatform.c
) 

target_link_libraries(example v_repExtRemoteApi pthread orocos-kdl
        ${OODL_YOUBOT_LIBRARIES}
        ${Boost_LIBRARIES}
)


