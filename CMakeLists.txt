# After installation this project can be found by 'find_package' command:
#
#    find_package(libcreate REQUIRED)
#    include_directores(${libcreate_INCLUDE_DIRS})
#    target_link_libraries(... ${libcreate_LIBRARIES})
#

cmake_minimum_required(VERSION 2.8.12)
project(libcreate)

add_compile_options(-Wall -Wextra -Wpedantic)

set(PACKAGE_VERSION 3.0.0)

option(LIBCREATE_BUILD_TESTS "Enable the build of tests." ON)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Threads REQUIRED)

# Default to C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

#########
# Build #
#########

set(LIBRARY_NAME create)

# Specify locations of header files
include_directories(
  include
)

# Declare cpp library
add_library(${LIBRARY_NAME} SHARED
  src/create.cpp
  src/serial.cpp
  src/serial_stream.cpp
  src/serial_query.cpp
  src/data.cpp
  src/packet.cpp
  src/types.cpp
)

# Manually link to thread library for build on ARM
if(THREADS_HAVE_PTHREAD_ARG)
  set_property(TARGET ${LIBRARY_NAME} PROPERTY COMPILE_OPTIONS "-pthread")
  set_property(TARGET ${LIBRARY_NAME} PROPERTY INTERFACE_COMPILE_OPTIONS "-pthread")
endif()

if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(${LIBRARY_NAME} "${CMAKE_THREAD_LIBS_INIT}")
endif()

# Link to Boost
target_link_libraries(${LIBRARY_NAME}
  ${Boost_LIBRARIES}
)

# Declare example executables
set(EXAMPLES
  battery_level
  bumpers
  cliffs
  drive_circle
  leds
  packets
  play_song
  wheeldrop
)

foreach(EXAMPLE ${EXAMPLES})
  add_executable(${EXAMPLE} examples/${EXAMPLE}.cpp)

  target_link_libraries(${EXAMPLE}
    ${Boost_LIBRARIES}
    ${LIBRARY_NAME}
  )
endforeach()

#################
# Configuration #
#################

# Install directories layout:
#   * <prefix>/lib/
#   * <prefix>/bin/
#   * <prefix>/include/
#   * <prefix>/lib/cmake/<PROJECT-NAME>
#   * <prefix>/share/<PROJECT_NAME>
set(LIB_INSTALL_DIR "lib")
set(BIN_INSTALL_DIR "bin")
set(INCLUDE_INSTALL_DIR "include")
set(CONFIG_INSTALL_DIR "${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME}")
set(SHARE_INSTALL_DIR "share/${PROJECT_NAME}")

set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(VERSION_CONFIG "${GENERATED_DIR}/${PROJECT_NAME}-config-version.cmake")
set(PROJECT_CONFIG "${GENERATED_DIR}/${PROJECT_NAME}-config.cmake")
set(TARGETS_EXPORT_NAME "${PROJECT_NAME}-targets")

include(CMakePackageConfigHelpers)

# Configure '<PROJECT-NAME>-config-version.cmake'
write_basic_package_version_file(
  "${VERSION_CONFIG}"
  VERSION "${PACKAGE_VERSION}"
  COMPATIBILITY SameMajorVersion
)

# Configure '<PROJECT-NAME>-config.cmake'
configure_package_config_file(
  "config.cmake.in"
  "${PROJECT_CONFIG}"
  INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}"
  PATH_VARS
    INCLUDE_INSTALL_DIR
    LIBRARY_NAME
)

###########
# Install #
###########

# Install targets
install(
  TARGETS ${LIBRARY_NAME}
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "${LIB_INSTALL_DIR}"
  ARCHIVE DESTINATION "${LIB_INSTALL_DIR}"
  RUNTIME DESTINATION "${BIN_INSTALL_DIR}"
  INCLUDES DESTINATION "${INCLUDE_INSTALL_DIR}"
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION ${INCLUDE_INSTALL_DIR}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Install config
install(
  FILES "${PROJECT_CONFIG}" "${VERSION_CONFIG}"
  DESTINATION "${CONFIG_INSTALL_DIR}"
)

# Install targets
install(
  EXPORT "${TARGETS_EXPORT_NAME}"
  DESTINATION "${CONFIG_INSTALL_DIR}"
)

# Install package.xml (for catkin)
install(
  FILES package.xml
  DESTINATION ${SHARE_INSTALL_DIR}
)

###########
# Testing #
###########

if(LIBCREATE_BUILD_TESTS)
  find_package(GTest)
  include_directories(${GTEST_INCLUDE_DIRS})
endif()

if(LIBCREATE_BUILD_TESTS AND ${GTEST_FOUND})
  message("GTest installation found. Building tests.")

  enable_testing()

  # Add tests
  set(LIBCREATE_TESTS
    test_create
    test_data
    test_packet
    test_robot_model
    test_serial_stream
    test_serial_query
  )

  foreach(LIBCREATE_TEST ${LIBCREATE_TESTS})
    add_executable(${LIBCREATE_TEST} tests/${LIBCREATE_TEST}.cpp)

    target_link_libraries(${LIBCREATE_TEST}
      ${LIBRARY_NAME}
      ${GTEST_LIBRARIES}
      gtest_main
    )

    add_test(
      NAME ${LIBCREATE_TEST}
      COMMAND ${LIBCREATE_TEST}
    )
  endforeach()
else()
  message("No GTest installation found. Skipping tests.")
endif()
