project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

option(USE_SIMULATOR "Uses term2 simulator when ON. Else uses matplotlib interface" OFF)

if(USE_SIMULATOR)
set(sources src/ukf.cpp src/main.cpp src/tools.cpp src/simconnect.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(UnscentedKF ${sources})
target_link_libraries(UnscentedKF z ssl uv uWS)

else()

set(sources src/ukf.cpp src/main.cpp src/tools.cpp src/mpconnect.cpp)
add_executable(UnscentedKF ${sources})
find_package(PythonLibs)
target_include_directories(UnscentedKF PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(UnscentedKF ${PYTHON_LIBRARIES})

endif()
