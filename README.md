# libcreate

C++ library for interfacing with iRobot's [Create 1 and 2](http://www.irobot.com/About-iRobot/STEM/Create-2.aspx) as well as most models of Roombas. This library forms the basis of the ROS driver in [create_autonomy](https://github.com/autonomylab/create_autonomy).

* Documentation: TODO
* Code API: TODO
* Protocol documentation:
  - [`V_1`](http://www.ecsl.cs.sunysb.edu/mint/Roomba_SCI_Spec_Manual.pdf) (Roomba 400 series )
  - [`V_2`](http://www.irobot.com/filelibrary/pdfs/hrd/create/Create%20Open%20Interface_v2.pdf) (Create 1, Roomba 500 series)
  - [`V_3`](https://cdn-shop.adafruit.com/datasheets/create_2_Open_Interface_Spec.pdf) (Create 2, Roomba 600-800 series)
* Author: [Jacob Perron](http://jacobperron.ca) ([Autonomy Lab](http://autonomylab.org), [Simon Fraser University](http://www.sfu.ca))
* Contributors: [Mani Monajjemi](http:mani.im), [Ben Wolsieffer](https://github.com/lopsided98)

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
* Inaccurate odometry angle for Create 1 ([#22](https://github.com/AutonomyLab/libcreate/issues/22))

## Build Status

![Build Status](https://api.travis-ci.org/AutonomyLab/libcreate.svg?branch=master)
