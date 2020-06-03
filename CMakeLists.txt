cmake_minimum_required(VERSION 3.4.3)

# General Configuration
project(prob-tool)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules")

# Add LLVM
find_package(LLVM REQUIRED CONFIG)
message(STATUS "Found LLVM ${LLVM_PACKAGE_VERSION}")
message(STATUS "Using LLVMConfig.cmake in: ${LLVM_DIR}")

add_definitions(${LLVM_DEFINITIONS})
include_directories(SYSTEM ${LLVM_INCLUDE_DIRS})

llvm_map_components_to_libnames(LLVM_LIBS Support Core Analysis ScalarOpts BitReader TransformUtils)

# Add Eigen
find_package(Eigen3 REQUIRED)
message(STATUS "Found Eigen3 in: ${EIGEN3_INCLUDE_DIR}")
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

if (NOT LLVM_ENABLE_RTTI)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti")
endif()

if (NOT LLVM_ENABLE_EH)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-exceptions")
endif()

file(GLOB SOURCES "src/*.cpp" "include/*.hpp")
add_executable(prob-tool ${SOURCES})

target_include_directories(prob-tool PRIVATE include)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -pedantic")
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address,undefined -fsanitize-address-use-after-scope -fno-omit-frame-pointer")

target_link_libraries(prob-tool ${LLVM_LIBS})
