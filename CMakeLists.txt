project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

add_definitions(-DUSE_ASIO -D_GNU_SOURCE)

set(sources src/main.cpp src/path_planner.cpp src/tools.cpp)

# Include directory for uws
include_directories(/usr/include/uWS) 


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

# Include path for uws
link_directories(/lib)


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl crypto boost_system libuWS)
