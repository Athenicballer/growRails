# Specify the minimum version of CMake required
cmake_minimum_required(VERSION 3.10)

# Define the project name
project(GrowerApp C CXX)

# Set the C++ standard to C++11 or higher
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# --- 1. Define the path to the PIGPIO source directory ---
# Assuming the pigpio source code is located in the sibling directory '../pigpio'
set(PIGPIO_SOURCE_DIR "${CMAKE_SOURCE_DIR}/../pigpio")

# --- 2. Build the PIGPIO library for linking ---
# NOTE: This is the most important change. We now add the pigpio source 
# as a sub-directory and instruct CMake to build it FIRST.
add_subdirectory(${PIGPIO_SOURCE_DIR} pigpio_build)

# --- 3. Find the required Pthread (threading) library ---
find_package(Threads REQUIRED)

# --- 4. Define the PIGPIO Include Directory ---
# The main pigpio.h is at the root of the source directory
set(PIGPIO_INCLUDE_DIR ${PIGPIO_SOURCE_DIR})

# --- 5. Find the Built PIGPIO Library ---
# The library target is called 'pigpio' in the original pigpio CMake/Makefile.
# We now link to the target that was just built in the previous steps.
# NOTE: We skip find_library() as we are linking directly to the build target.

# --- 6. Add the executable target ---
add_executable(${PROJECT_NAME} main.cpp)

# --- 7. Link the executable with the pigpio build target ---
target_link_libraries(${PROJECT_NAME}
    # This links against the pigpio library target built by the add_subdirectory command
    pigpio 
    Threads::Threads # Link against Pthreads
)

# --- 8. Add the pigpio include path for the compiler ---
target_include_directories(${PROJECT_NAME} PRIVATE
    ${PIGPIO_INCLUDE_DIR}
)

# This entire setup eliminates the need to manually copy pigpio.h and libpigpio.a 
# into the sysroot, as CMake now handles building pigpio first and linking to it.
