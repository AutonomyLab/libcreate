# After installation this project can be found  by 'find_package(... CONFIG)' command:
#
#    find_package(libcreate CONFIG REQUIRED)
#    target_link_libraries(... libcreate::create)
#
# Reference for this file:
#   https://github.com/forexample/package-example/blob/cf2ea1d6a209fb9eca2ab83fdd0ac15fe4d3e807/Foo/CMakeLists.txt

cmake_minimum_required(VERSION 2.8.12)
project(libcreate)

set(package_version 1.2.0)

find_package(Boost REQUIRED system thread)
find_package(Threads REQUIRED)

#########
# Build #
#########

# Specify locations of header files
include_directories(
  include
)

# Declare cpp library
add_library(create SHARED
  src/create.cpp
  src/serial.cpp
  src/serial_stream.cpp
  src/serial_query.cpp
  src/data.cpp
  src/packet.cpp
  src/types.cpp
)

# Global includes. Used by all targets
#   * header can be included by C++ code `#include <create/create.h>`
target_include_directories(
  create PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
)

# Manually link to thread library for build on ARM
if(THREADS_HAVE_PTHREAD_ARG)
  set_property(TARGET create PROPERTY COMPILE_OPTIONS "-pthread")
  set_property(TARGET create PROPERTY INTERFACE_COMPILE_OPTIONS "-pthread")
endif()

if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(create "${CMAKE_THREAD_LIBS_INIT}")
endif()

# Link to Boost
target_link_libraries(create
  ${Boost_LIBRARIES}
)

# Declare example executables
add_executable(create_demo examples/create_demo.cpp)
add_executable(bumper_example examples/bumper_example.cpp)
add_executable(odom_example examples/odom_example.cpp)

# Specify libraries to link executable targets against
target_link_libraries(create_demo
  ${Boost_LIBRARIES}
  create
)
target_link_libraries(bumper_example
  ${Boost_LIBRARIES}
  create
)
target_link_libraries(odom_example
  ${Boost_LIBRARIES}
  create
)

###########
# Install #
###########

# Layout. This works for all platforms:
#   * <prefix>/lib/cmake/<PROJECT-NAME>
#   * <prefix>/lib/
#   * <prefix>/include/
set(config_install_dir "lib/cmake/${PROJECT_NAME}")
set(include_install_dir "include")

set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")

# Configuration
set(version_config "${generated_dir}/${PROJECT_NAME}-config-version.cmake")
set(project_config "${generated_dir}/${PROJECT_NAME}-config.cmake")
set(targets_export_name "${PROJECT_NAME}-targets")
set(namespace "${PROJECT_NAME}::")

# Include module with function 'write_basic_package_version_file'
include(CMakePackageConfigHelpers)

# Configure '<PROJECT-NAME>-config-version.cmake'
write_basic_package_version_file(
  "${version_config}"
  VERSION "${package_version}"
  COMPATIBILITY SameMajorVersion
)

# Configure '<PROJECT-NAME>-config.cmake'
# Use variables:
#   * targets_export_name
#   * PROJECT_NAME
configure_package_config_file(
  "config.cmake.in"
  "${project_config}"
  INSTALL_DESTINATION "${config_install_dir}"
)

# Install targets
install(
  TARGETS create
  EXPORT "${targets_export_name}"
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin"
  INCLUDES DESTINATION "${include_install_dir}"
)

# Install headers
install(DIRECTORY include/create
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Install config
#   * <prefix>/lib/cmake/libcreate/libcreate-config.cmake
#   * <prefix>/lib/cmake/libcreate/libcreate-config-version.cmake
install(
  FILES "${project_config}" "${version_config}"
  DESTINATION "${config_install_dir}"
)

# Install config
#   * <prefix>/lib/cmake/libcreate/libcreate-targets.cmake
install(
  EXPORT "${targets_export_name}"
  NAMESPACE "${namespace}"
  DESTINATION "${config_install_dir}"
)

# Install package.xml (for catkin)
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)
