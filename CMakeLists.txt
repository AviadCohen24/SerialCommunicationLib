cmake_minimum_required(VERSION 3.10)
project(SerialComm)

set(CMAKE_CXX_STANDARD 14)

# MSVC static runtime linking
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# Manually set Boost include directory and library paths
set(BOOST_INCLUDEDIR "${CMAKE_SOURCE_DIR}/boost/include")
set(BOOST_LIBRARYDIR "${CMAKE_SOURCE_DIR}/boost/lib")

include_directories(${BOOST_INCLUDEDIR})

add_library(serial_comm SHARED serial_comm.cpp)

# Warning flags
if(MSVC)
  target_compile_options(serial_comm PRIVATE /W4 /WX)
else()
  target_compile_options(serial_comm PRIVATE -Wall -Wextra -pedantic -Werror)
endif()

# Platform-specific library names
if(WIN32)
    set(BOOST_SYSTEM_LIB "${BOOST_LIBRARYDIR}/libboost_system-vc141-mt-x64-1_65.lib")
    set(BOOST_THREAD_LIB "${BOOST_LIBRARYDIR}/libboost_thread-vc141-mt-x64-1_65.lib")
else()
    set(BOOST_SYSTEM_LIB "${BOOST_LIBRARYDIR}/libboost_system.a")
    set(BOOST_THREAD_LIB "${BOOST_LIBRARYDIR}/libboost_thread.a")
endif()

# Linking Boost libraries
target_link_libraries(serial_comm ${BOOST_SYSTEM_LIB} ${BOOST_THREAD_LIB})
