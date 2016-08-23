cmake_minimum_required(VERSION 2.8.3)
project(libcreate)

find_package(Boost REQUIRED system thread)
find_package(Threads REQUIRED)

## Specify additional locations of header files
include_directories(
  include
)

## Declare cpp library
add_library(create
  src/create.cpp
  src/serial.cpp
  src/serial_stream.cpp
  src/serial_query.cpp
  src/data.cpp
  src/packet.cpp
  src/types.cpp
)

if(THREADS_HAVE_PTHREAD_ARG)
  set_property(TARGET create PROPERTY COMPILE_OPTIONS "-pthread")
  set_property(TARGET create PROPERTY INTERFACE_COMPILE_OPTIONS "-pthread")
endif()

if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(create "${CMAKE_THREAD_LIBS_INIT}")
endif()

target_link_libraries(create
  ${Boost_LIBRARIES}
)

## Declare example executables
add_executable(create_demo examples/create_demo.cpp)
add_executable(bumper_example examples/bumper_example.cpp)
add_executable(odom_example examples/odom_example.cpp)

## Specify libraries to link a library or executable target against
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

## Install
install(TARGETS create DESTINATION lib)
install(FILES
        include/create/create.h
        include/create/serial.h
        include/create/types.h
        include/create/data.h
        include/create/packet.h
        include/create/util.h
        DESTINATION include/create)
