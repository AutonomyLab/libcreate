^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libcreate
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1.6.1 (2018-04-21)
------------------
* Build and install gtest as part of CI
* Update README with instructions for building and running unit tests
* Remove external cmake project for gtest
  Now only build tests if a gtest installation already exists on the system. This should expedite time to build for users that do not care about building/running unit tests and also eliminates the need for internet access when building.
* Add test depend to gtest in package.xml
* Contributors: Jacob Perron

1.6.0 (2018-04-07)
------------------
* Add unit tests (gtests)
* Refactor Packet API
    * Declare setData member as protected
    * Rename 'setTempData' to 'setDataToValidate'
* Remove redundant packets from Data constructor
* Updated setDigits function API comments
    * added HTML to adjust for spacing in diagram, showing the proper ordering of segments.
* Update examples
    * More concise and focusing on individual features:
        * Battery level
        * Bumpers
        * Drive circle
        * LEDs
        * Serial packets
        * Play song
        * Wheeldrop
* Update README
* Refactor cmake files
* Contributors: Jacob Perron, K.Moriarty

1.5.0 (2017-12-17)
------------------
* Add APIs for getting the measured velocities of the wheels
* Add ability to drive the wheels with direct pwm duty
* Update documentation
* Add mainpage.dox
* Use package.xml format 2
* Add doxygen as doc dependency
* Contributors: Erik Schembor, Jacob Perron

1.4.0 (2016-10-16)
------------------
* Switch to trusty for CI
* Set mimumum cmake version to 2.8.12
* Update CMakeLists.txt configuration and install rules
* Add package.xml
* Add config.cmake.in
* Contributors: Jacob Perron

1.3.0 (2016-08-23)
------------------
* Add support for early model Roomba 400s and other robots using the original SCI protocol.
* Expose individual wheel distances and requested velocities. Fix wheel distance calculation for the Create 1.
* Manually link to thread library. This allows libcreate to build on ARM.
* Fix odometry inversion for Create 1.
* Contributors: Ben Wolsieffer, Jacob Perron

1.2.1 (2016-04-30)
------------------
* Make velocity relative to base frame, not odometry frame
* Contributors: Jacob Perron

1.2.0 (2016-04-15)
------------------
* Add covariance info to Pose and Vel
* Fix getMode bug
* Contributors: Jacob Perron

1.1.1 (2016-04-07)
------------------
* Fix odometry sign error
* Add warning in code regarding Create 1 odometry issue
* Add odom_example.cpp
* Contributors: Jacob Perron

1.1.0 (2016-04-02)
------------------
* Add API to get light sensor signals
* Contributors: Jacob Perron

1.0.0 (2016-04-01)
------------------
* Fix odometry for Create 1
* Fix odom angle sign error
* Convert units to base units
* Implement 'getMode'
* Rename 'isIRDetect*' functions to 'isLightBumper*'
* Documentation / code cleanup
* Add function 'driveRadius'
* Add function 'isVirtualWall'
* Fix sign error on returned 'current' and 'temperature'
* Contributors: Jacob Perron

0.1.1 (2016-03-25)
------------------
* Fix odometry bug
* Contributors: Jacob Perron

0.1.0 (2016-03-24)
------------------
* Add enum of special IR characters
* Fix bug: convert distance measurement to meters
* Add support for first generation Create (Roomba 400 series)
* Fix bug: Too many packets requested corrupting serial buffer
* Expose functions for getting number of corrupt packets and total packets in Create class
* Add getters for number of corrupt and total packets received over serial
* Update README.md
* Added build badge
* Added CI (travis)
* Instantaneous velocity now available
* Contributors: Jacob Perron
