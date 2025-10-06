cmake_minimum_required(VERSION 3.10)
project(cricket)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set CMake Policies
set(CMAKE_POLICY_DEFAULT_CMP0167 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0175 OLD)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---- CPM bootstrap ----
set(CPM_DOWNLOAD_VERSION 0.38.6)
set(CPM_DOWNLOAD_LOCATION "${CMAKE_BINARY_DIR}/cmake/CPM.cmake")
if(NOT EXISTS "${CPM_DOWNLOAD_LOCATION}")
  file(DOWNLOAD
    "https://github.com/cpm-cmake/CPM.cmake/releases/download/v${CPM_DOWNLOAD_VERSION}/CPM.cmake"
    "${CPM_DOWNLOAD_LOCATION}"
    EXPECTED_HASH SHA256=c4f3c2a07a2d7d1e2e2c0e5ed1b4b7f3f0fc2d1d28b0a0b1b7f3c2c92a9b1f5a # (optional)
    TLS_VERIFY ON)
endif()
include("${CPM_DOWNLOAD_LOCATION}")
# -----------------------


if(NOT DEFINED ENV{CPM_SOURCE_CACHE})
  set(ENV{CPM_SOURCE_CACHE} ${CMAKE_CURRENT_SOURCE_DIR}/.cpm-cache)
endif()
include(cmake/CPM.cmake)

# Find required packages
find_package(PkgConfig REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)
find_package(fmt REQUIRED)
find_package(inja REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

# Find CppAD
find_package(cppad QUIET)
if(NOT cppad_FOUND)
    pkg_check_modules(cppad REQUIRED cppad)
endif()
message(STATUS "Using CppAD include dirs: ${cppad_INCLUDE_DIRS}")
message(STATUS "Using CppAD lib : ${cppad_LIBRARIES}")
message(STATUS "Using CppAD lib dirs: ${cppad_LIBRARY_DIRS}")

# Setup CppADCG
CPMAddPackage(
  URI "gh:joaoleal/CppADCodeGen@2.5.0"
  PATCHES CppADCodeGen.patch
  OPTIONS "ENABLE_THREAD_POOL_TESTS Off"
)

list(APPEND cppadcg_INCLUDE_DIRS
  "${cppadcg_SOURCE_DIR}/include"
  "${cppadcg_BINARY_DIR}/include")

# Create the executable
add_executable(fkcc_gen src/fkcc_gen.cc)

# Set include directories
target_include_directories(fkcc_gen PRIVATE
    ${pinocchio_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${cppad_INCLUDE_DIRS}
    ${cppadcg_INCLUDE_DIRS}
)

target_link_directories(fkcc_gen PRIVATE ${cppad_LIBRARY_DIRS})

# Link libraries
target_link_libraries(fkcc_gen
    pinocchio::pinocchio
    ${Boost_LIBRARIES}
    ${cppad_LIBRARIES}
    pantor::inja
    nlohmann_json::nlohmann_json
    CGAL::CGAL
    fmt::fmt
)

# Add compiler definitions
target_compile_definitions(fkcc_gen PRIVATE
    ${pinocchio_CFLAGS_OTHER}
)