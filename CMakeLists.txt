cmake_minimum_required (VERSION 3.4.3)
project (pdc_llvm_support)
set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
set (CMAKE_CXX_EXTENSIONS OFF)

find_package (LLVM REQUIRED CONFIG)

message (STATUS "Found LLVM ${LLVM_PACKAGE_VERSION}")
message (STATUS "Using LLVMConfig.cmake in: ${LLVM_DIR}")

add_definitions (${LLVM_DEFINITIONS})
include_directories (${LLVM_INCLUDE_DIRS})

llvm_map_components_to_libnames (LLVM_LIBS support core scalarOpts bitReader)

#if (NOT LLVM_ENABLE_RTTI)
#  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti")
#endif()
#
#if (NOT LLVM_ENABLE_EH)
#  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-exceptions")
#endif()

file (GLOB SOURCES "src/*.cpp" "include/*.hpp")
add_executable (prob-tool ${SOURCES})
target_include_directories(prob-tool PRIVATE include)
target_compile_options(prob-tool PUBLIC ${CMAKE_C_FLAGS} -Wall -Werror -pedantic)

target_link_libraries (prob-tool ${LLVM_LIBS})
