# libcreate

C++ library for interfacing with iRobot's [Create 2](http://www.irobot.com/About-iRobot/STEM/Create-2.aspx).

* Documentation: TODO
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

![Build Status](https://api.travis-ci.org/AutonomyLab/libcreate.svg?branch=master)
