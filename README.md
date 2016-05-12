Skytraq GPS Driver
==================

(THIS CODE IS A WORK IN PROGRESS)

This C++ driver is written for the Skytraq S1315F-RAW GPS receiver using the Skytraq Binary protocol. NMEA outputs are also available on this receiver but are not supported in this driver.

It is written as a standlone library which depends on [Boost](http://http://www.boost.org) and a simple cross-platform serial port library [serial port library](https://github.com/wjwwood/serial).  It uses [catkin](http://wiki.ros.org/catkin) for the build system.

# Dependencies

* Cmake
* catkin
* Boost
* serial

# Create a catkin workspace

    mkdir sytraq_ws
    cd skytraq_ws
    mkdir src

# Installation 
Clone the serial repo into ~/skytraq_ws/src:

    git clone git://github.com/wjwwood/serial.git
	  
Clone the skytraq repo into ~/skytraq_ws/src:
  
    git clone git://github.com/chris5108/skytraq.git

# Build

    cd ~/skytraq_ws
    catkin_make
    

    
    
