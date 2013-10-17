cmake_minimum_required (VERSION 2.6) 
project (HELLO) 


set(VREP_DIR "/opt/v-rep")

set(CMAKE_C_FLAGS "-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -Wno-write-strings")


include_directories (
    include
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

target_link_libraries(example remoteApi v_repExtRemoteApi pthread)


