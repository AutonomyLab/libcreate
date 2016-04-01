# libcreate

C++ library for interfacing with iRobot's [Create 1 and 2](http://www.irobot.com/About-iRobot/STEM/Create-2.aspx). This library forms the basis of the ROS driver in [create_autonomy](https://github.com/autonomylab/create_autonomy).

* Documentation: TODO
* Code API: TODO
* Author: [Jacob Perron](http://jacobperron.ca) ([Autonomy Lab](http://autonomylab.org), [Simon Fraser University](http://www.sfu.ca))
* Contributors: [Mani Monajjemi](http:mani.im)

## Dependencies

* [Boost System Library](http://www.boost.org/doc/libs/1_59_0/libs/system/doc/index.html)
* [Boost Thread Library](http://www.boost.org/doc/libs/1_59_0/doc/html/thread.html)

## Install

* `cmake CMakeLists.txt`
* `make`
* `sudo make install`

## Example

See source for examples.
 
Example compile line: `g++ create_demo.cpp -lcreate -lboost_system -lboost_thread`

## Bugs

* _Clock_ and _Schedule_ button presses are not detected. This is a known problem to the developers at iRobot.

## Build Status

![Build Status](https://api.travis-ci.org/AutonomyLab/libcreate.svg?branch=master)
