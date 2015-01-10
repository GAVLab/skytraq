Skytraq GPS Driver
==================

This driver is written for the Skytraq S1315F-RAW GPS receiver using the Skytraq Binary protocol. NMEA outputs are also available on this receiver but are not supported in this driver.

The Novatel driver is written as a standlone library which depends on [Boost](http://http://www.boost.org) and a simple cross-platform serial port library [serial port library](https://github.com/wjwwood/serial).  It uses [Cmake](http://http://www.cmake.org) for the build system.

# Installation 
The serial library can be installed by:

    git clone git://github.com/wjwwood/serial.git
	  cd serial
	  git checkout fuerte
	  make
	  sudo make install
	  
The skytraq library can be installed by:
  
    git clone git://github.com/chris5108/skytraq.git
    cd skytraq
    cmake ./
    make
    sudo make install
    

    
    
