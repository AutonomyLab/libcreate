# libcreate #

C++ library for interfacing with iRobot's [Create 1 and 2](http://www.irobot.com/About-iRobot/STEM/Create-2.aspx) as well as most models of Roomba. [create_autonomy](http://wiki.ros.org/create_autonomy) is a [ROS](http://www.ros.org/) wrapper for this library.

* [Code API](http://docs.ros.org/kinetic/api/libcreate/html/index.html)
* Protocol documentation:
  - [`V_1`](https://drive.google.com/open?id=0B9O4b91VYXMdUHlqNklDU09NU0k) (Roomba 400 series )
  - [`V_2`](https://drive.google.com/open?id=0B9O4b91VYXMdMmFPMVNDUEZ6d0U) (Create 1, Roomba 500 series)
  - [`V_3`](https://drive.google.com/open?id=0B9O4b91VYXMdSVk4amw1N09mQ3c) (Create 2, Roomba 600-800 series)
* Author: [Jacob Perron](http://jacobperron.ca) ([Autonomy Lab](http://autonomylab.org), [Simon Fraser University](http://www.sfu.ca))
* Contributors: [Mani Monajjemi](http:mani.im), [Ben Wolsieffer](https://github.com/lopsided98)

## Build Status ##

![Build Status](https://api.travis-ci.org/AutonomyLab/libcreate.svg?branch=master)

## Dependencies ##

* [Boost System Library](http://www.boost.org/doc/libs/1_59_0/libs/system/doc/index.html)
* [Boost Thread Library](http://www.boost.org/doc/libs/1_59_0/doc/html/thread.html)
* [Optional] [googletest](https://github.com/google/googletest)

### Install ###

        sudo apt-get install build-essential cmake libboost-system-dev libboost-thread-dev

        # Optionally, install gtest for building unit tests
        sudo apt-get install libgtest-dev
        cd /usr/src/gtest
        sudo cmake CMakeLists.txt
        sudo make
        sudo cp *.a /usr/lib

#### Serial Permissions ####

User permission is requried to connect to Create over serial. You can add your user to the dialout group to get permission:

        sudo usermod -a -G dialout $USER

Logout and login again for this to take effect.

## Build ##

Note, the examples found in the "examples" directory are built with the library.

#### cmake ####

        git clone https://github.com/AutonomyLab/libcreate.git
        cd libcreate
        mkdir build && cd build
        cmake ..
        make -j

#### catkin ####

Requires [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/).

        mkdir -p create_ws/src
        cd create_ws
        catkin init
        cd src
        git clone https://github.com/AutonomyLab/libcreate.git
        catkin build

## Running Tests ##

To run unit tests, execute the following in the build directory:

        make test

## Known Issues ##

* _Clock_ and _Schedule_ buttons are not functional. This is a known bug related to the firmware.
* Inaccurate odometry angle for Create 1 ([#22](https://github.com/AutonomyLab/libcreate/issues/22))
