cmake_minimum_required(VERSION 2.8.3)
project(ArmourShoot)

#cmake_policy(SET CMP0023 OLD)
#cmake_policy(SET CMP0022 OLD)

set(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPATH=\"\\\"${PROJECT_SOURCE_DIR}\\\"\"")

set(GXI_API_DIR "/usr/lib/")

#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)

message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

include_directories(
                    #${EIGEN3_INCLUDE_DIR}
                    ${OpenCV_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include/armour
                    ${PROJECT_SOURCE_DIR}/include/IndusCamera
                    ${PROJECT_SOURCE_DIR}/include/serial
                    ${PROJECT_SOURCE_DIR}/include/
                    ${PROJECT_SOURCE_DIR}/include/classifier
)

add_subdirectory(src)

add_executable(ArmourShoot main.cpp)
target_link_libraries(ArmourShoot src)

