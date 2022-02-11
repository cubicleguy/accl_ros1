# README for Epson Accelerometer Driver for ROS1 Node

## What is this repository for?

* This code provides interface between Epson Accelerometer and ROS1 using either SPI  or UART interface.
* For using the SPI interface, this code uses the [Unofficial wiringPi library](https://github.com/WiringPi/WiringPi/) for accessing GPIO and SPI functions on the Raspberry Pi platform running Ubuntu Linux + ROS Melodic
  * The src/epson_accl_spi_node.cpp is the ROS1 C++ wrapper used to communicate with ROS1
* For using the UART interface, the UART connection can be either direct or by USB-serial converter such as FTDI bridge ICs.
  * The src/epson_accl_uart_node.cpp is the ROS C++ wrapper used to communicate with ROS1
* The other source files in src/ are based on the C driver released by Epson:
  [Epson Accelerometer Linux User-space Driver Example](https://vdc.epson.com/imu-products/accelerometers)
* Information about ROS1, and tutorials can be found: [ROS.org](https://wiki.ros.org/)


## What kind of hardware or software will I likely need?

* ROS Melodic, Lunar, Kinetic, Indigo (via download) [ROS.org](https://www.ros.org)
  * Installation guide [ROS.org](https://wiki.ros.org/ROS/Installation)
* This software was developed and tested on the following:
```
  ROS1:        Melodic
  Description: Ubuntu 18.04.5 LTS
  Release:     18.04
  Codename:    bionic
```
* Epson Accelerometer [ACCL models](
https://global.epson.com/products_and_drivers/sensing_system/acc/)

### SPI
* The embedded Linux host system will need the SPI interface (SCLK, MISO, MOSI) and 3 GPIOs (CS#, RST#, DRDY) enabled prior to using this software.
  - For Raspberry Pi, the SPI interface must already be enabled using raspi-config
  - This code uses a separate GPIO to manually toggle SCS# chipselect instead of the chipselect assigned to the RPI SPI interface
* Epson Breakout evaluation board or some equivalent to connect to ROS host (SPI & GPIOs) [See Evaluation Boards](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/)

### UART
* Any generic Linux host system that has an available tty UART port to connect to the Epson ACCL
* Epson USB evaluation board or equivalent FTDI USB-Serial interface to connect to ROS host (tty/serial) [See Evaluation Boards](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/)


## How do I use the driver?

* This code assumes that the user is familiar with building ROS1 packages using the catkin build process.
* This is *NOT* detailed instructions describing step by step procedures on how to build and install this ROS1 driver.
* Please refer to the ROS.org website for more detailed instructions on configuring the ROS environment & the ROS package build process. [ROS.org](https://wiki.ros.org/ROS/Tutorials)
* If the ACCL model is unchanged since last catkin build, then any subsequent changes to ACCL settings can be done by editing the ACCL model specific .launch file.
* The ACCL model-specific launch file should only be used with the matching catkin-built executable of the same ACCL model.
* *NOTE* Do not mix ACCL model launch files & ACCL model catkin built binaries.


## How do I use the driver if usleep() is not supported for time delays?

* **NOTE:** In the hcl_rpi.c & hcl_linux.c, there are wrapper functions for time delays in millisecond and microseconds using seDelayMS() and seDelayMicroSecs(), respectively.
* On embedded Linux platforms, these may need to be redirected to HW platform specific delay routines.
* For example on RaspberryPi, the time delay functions for millisecond and microseconds can be redirected to WiringPi library delay() and delayMicroseconds(), respectively.
* If a hardware delay is not available from a library, then a software delay loop is possible but not preferred.


## How do I use the driver with GPIOs to control ACCL RESET#, DRDY, EXT pins?

* When using this driver to connect using the SPI interface, the use of GPIO pins for connecting to the ACCL SCS# and DRDY is **mandatory** (RESET# is recommended, EXT is optional).
* When using this driver to connect using the UART interface, the use of GPIO pins for connecting to the ACCL RESET#, EXT, or DRDY is purely **optional** and mainly intended for embedded Linux platforms (non-PC based).
* When possible, connecting the RESET# is recommended to force Hardware Reset during every ACCL initialization for better robustness.
* This code is structured to easily redirect to low-level hardware GPIO function calls for easy implementation such as RaspberryPi.


### Modifying hcl_xxx.c
* There is no standard method to implement GPIO connections on embedded Linux platform, but the following files typically need changes:

```
  src/hcl_rpi.c
  src/hcl_gpio_rpi.c
  src/hcl_gpio.h
```

* Typically, an external library needs to be invoked to initialize & enable GPIO HW functions.

* This typically requires changes to hcl_[platform].c, i.e. Use hcl_rpi.c as a template

  - add #include to external library near the top of hcl_[platform].c
  - add the initialization call inside the seInit() function in hcl_[platform].c

For example on an Raspberry Pi, the following changes can be made to hcl_[platform].c:

```
  .
  .
  .
  #include <stdint.h>
  #include <stdio.h>
  #include <wiringPi.h>  // <== Added external library

  int seInit(void)
  {
    // Initialize wiringPi libraries                                                   // <== Added
    printf("\r\nInitializing libraries...");                                           // <== Added
    if(wiringPiSetupGpio() != 0) {                                                     // <== Added external library initialization
      printf("\r\nError: could not initialize wiringPI libraries. Exiting...\r\n");    // <== Added
      return NG;                                                                       // <== Added
    }                                                                                  // <== Added
    printf("...done.");

    return OK;
  }
  .
  .
  .

```

### Modifying hcl_gpio.h
* Typically, the GPIO pins need to be assigned according to pin numbering specific to the HW platform.

* This typically requires changes to hcl_gpio.h

For example on an Raspberry Pi, the following changes to hcl_gpio.h with the following pin mapping:

```
    Epson ACCL                   Raspberry Pi
    ---------------------------------------------------
    EPSON_RESET                 RPI_GPIO_P1_15 (GPIO22) Output
    EPSON_DRDY                  RPI_GPIO_P1_18 (GPIO24) Input
    EPSON_CS                    RPI_GPIO_P1_16 (GPIO23) Output

```
Note: The RPI SPI0_cs0 is not connected. Chip select is being manually controlled via GPIO on P1_16. 

```
  // Prototypes for generic GPIO functions
  int gpioInit(void);
  int gpioRelease(void);

  void gpioSet(uint8_t pin);
  void gpioClr(uint8_t pin);
  uint8_t gpioGetPinLevel(uint8_t pin);

  #define RPI_GPIO_P1_15              22                    // <== Added
  #define RPI_GPIO_P1_16              23                    // <== Added
  #define RPI_GPIO_P1_18              24                    // <== Added

  #define EPSON_RESET                 RPI_GPIO_P1_15        // <== Added
  #define EPSON_CS                    RPI_GPIO_P1_16        // <== Added 
  #define EPSON_DRDY                  RPI_GPIO_P1_18        // <== Added
  .
  .
  .
```


### Modifying hcl_gpio_xxx.c
* Typically, the external library will have GPIO pin control functions such as set_output, set_input, set, reset, read_pin_level, etc...

* This requires changes to hcl_gpio_rpi.c

  - Redirect function calls in hcl_gpio.c for gpioInit(), gpioRelease(), gpioSet(), gpioClr(), gpioGetPinLevel() to equivalent external library pin control functions.

  - For example on an Raspberry Pi, the following changes to hcl_gpio_rpi.c:

```
  #include "hcl.h"
  #include "hcl_gpio.h"
  #include <wiringPi.h>                         // <== Added external library
  .
  .
  .

  int gpioInit(void)
  {
    pinMode(EPSON_RESET, OUTPUT);               // <== Added external call RESET Output Pin
    pinMode(EPSON_CS, OUTPUT);                  // <== Added external call CS# Output Pin
    pinMode(EPSON_DRDY, INPUT);                 // <== Added external call DRDY Input Pin
    pullUpDnControl(EPSON_DRDY, PUD_OFF) ;      // <== Added external call Disable any internal pullup or pulldown resistance

    return OK;
  }
  .
  .
  .
  int gpioRelease(void)
  {
    return OK;
  }
  .
  .
  .
  void gpioSet(uint8_t pin)
  {
    digitalWrite(pin, HIGH);                    // <== Added external call set pin HIGH
  }

  .
  .
  .
  void gpioClr(uint8_t pin)
  {
    digitalWrite(pin, LOW);                     // <== Added external call set pin LOW
  }

  .
  .
  .
  uint8_t gpioGetPinLevel(uint8_t pin)
  {
    return (digitalRead(pin));                  // <== Added external call to return pin state of input pin
  }

  .
  .
  .
```


## How do I build, install, run this ROS1 package?

- The Epson ACCL ROS1 driver is designed to build in the ROS catkin build environment.
- Therefore, a functional catkin workspace in ROS1 is a prerequisite.
- Refer to the ROS1 Tutorials for more info: [ROS1 Tutorial](https://wiki.ros.org/ROS/Tutorials)

### Changing the serial interface type: UART or SPI
**NOTE**: When re-building this software after changing the *interface type* between SPI & UART,
it is necessary to delete the previous library file incase catkin_make does not properly detect 
that the interface has been switched, and incorrectly builds with the previously built library.

1. To do this, go into the <catkin_workspace>/devel/lib folder
2. Delete the **libepson_accl_ros_driver_lib.so** file & epson_accl_ros_driver folder

### Normal build process
- For more information on ROS & catkin setup refer to
[Installing and Configuring ROS Environment](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).


1. Place this package (including folders) into a new folder within your catkin workspace "src" folder.
   For example, we recommend using the folder name "accl_ros_node"
```
   <catkin_workspace>/src/accl_ros_node/ <-- place files here
```
2. Modify the CMakeLists.txt:
   Refer to the comment lines inside the CMakeLists.txt for additional info.
   - select the desired Epson ACCL model (accl_model)
   - select the serial interface type (interface)

   **NOTE:** You *MUST* re-build using catkin build when making any changes in the CmakeLists.txt

3. From the catkin workspace folder run "catkin_make" to build all ROS1 packages located in the <catkin_workspace>/src/ folder.

```
   <catkin_workspace>/catkin_make
```
   Re-run the above "catkin_make" command to rebuild the driver after making any changes to the CMakeLists.txt or any of the .c or .cpp or .h source files.
   It is not necessary to "catkin_make" if changes are only made to the launch files

   **NOTE:** It is recommended to change ACCL settings by editing the parameters in the launch file, wherever possible, instead of modifying the .c or .cpp source files directly

### Example console output of catkin build for A352 UART:
```
guest@guest-desktop:~/catkin_ws$ catkin_make
Base path: /home/guest/catkin_ws
Source space: /home/guest/catkin_ws/src
Build space: /home/guest/catkin_ws/build
Devel space: /home/guest/catkin_ws/devel
Install space: /home/guest/catkin_ws/install
####
#### Running command: "make cmake_check_build_system" in "/home/guest/catkin_ws/build"
####
make: Warning: File 'Makefile' has modification time 13975 s in the future
-- Using CATKIN_DEVEL_PREFIX: /home/guest/catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /home/guest/catkin_ws/devel;/opt/ros/melodic
-- This workspace overlays: /home/guest/catkin_ws/devel;/opt/ros/melodic
-- Found PythonInterp: /usr/bin/python2 (found suitable version "2.7.17", minimum required is "2")
-- Using PYTHON_EXECUTABLE: /usr/bin/python2
-- Using Debian Python package layout
-- Using empy: /usr/bin/empy
-- Using CATKIN_ENABLE_TESTING: ON
-- Call enable_testing()
-- Using CATKIN_TEST_RESULTS_DIR: /home/guest/catkin_ws/build/test_results
-- Found gtest sources under '/usr/src/googletest': gtests will be built
-- Found gmock sources under '/usr/src/googletest': gmock will be built
-- Found PythonInterp: /usr/bin/python2 (found version "2.7.17")
-- Using Python nosetests: /usr/bin/nosetests-2.7
-- catkin 0.7.28
-- BUILD_SHARED_LIBS is on
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 1 packages in topological order:
-- ~~  - epson_accl_ros_driver
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'epson_accl_ros_driver'
-- ==> add_subdirectory(accl_ros_node)
-- Boost version: 1.65.1
-- Found the following Boost libraries:
--   system
---- Building for ACCL Model: A352
---- Building for interface: UART
---- Building for platform: NONE
-- Configuring done
-- Generating done
-- Build files have been written to: /home/guest/catkin_ws/build
make: warning:  Clock skew detected.  Your build may be incomplete.
####
#### Running command: "make -j4 -l4" in "/home/guest/catkin_ws/build"
####
-- Using CATKIN_DEVEL_PREFIX: /home/guest/catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /home/guest/catkin_ws/devel;/opt/ros/melodic
-- This workspace overlays: /home/guest/catkin_ws/devel;/opt/ros/melodic
-- Found PythonInterp: /usr/bin/python2 (found suitable version "2.7.17", minimum required is "2")
-- Using PYTHON_EXECUTABLE: /usr/bin/python2
-- Using Debian Python package layout
-- Using empy: /usr/bin/empy
-- Using CATKIN_ENABLE_TESTING: ON
-- Call enable_testing()
-- Using CATKIN_TEST_RESULTS_DIR: /home/guest/catkin_ws/build/test_results
-- Found gtest sources under '/usr/src/googletest': gtests will be built
-- Found gmock sources under '/usr/src/googletest': gmock will be built
-- Found PythonInterp: /usr/bin/python2 (found version "2.7.17")
-- Using Python nosetests: /usr/bin/nosetests-2.7
-- catkin 0.7.28
-- BUILD_SHARED_LIBS is on
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 1 packages in topological order:
-- ~~  - epson_accl_ros_driver
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'epson_accl_ros_driver'
-- ==> add_subdirectory(accl_ros_node)
-- Boost version: 1.65.1
-- Found the following Boost libraries:
--   system
---- Building for ACCL Model: A352
---- Building for interface: UART
---- Building for platform: NONE
-- Configuring done
-- Generating done
-- Build files have been written to: /home/guest/catkin_ws/build
Scanning dependencies of target epson_accl_ros_driver_lib
make[2]: Warning: File '/home/guest/catkin_ws/src/accl_ros_node/src/hcl_linux.c' has modification time 3366415 s in the future
[ 33%] Building C object accl_ros_node/CMakeFiles/epson_accl_ros_driver_lib.dir/src/hcl_linux.c.o
[ 33%] Building C object accl_ros_node/CMakeFiles/epson_accl_ros_driver_lib.dir/src/hcl_gpio.c.o
[ 33%] Building C object accl_ros_node/CMakeFiles/epson_accl_ros_driver_lib.dir/src/hcl_uart.c.o
[ 44%] Building C object accl_ros_node/CMakeFiles/epson_accl_ros_driver_lib.dir/src/accel_epsonCommon.c.o
[ 66%] Building C object accl_ros_node/CMakeFiles/epson_accl_ros_driver_lib.dir/src/accel_epsonUart.c.o
[ 66%] Building C object accl_ros_node/CMakeFiles/epson_accl_ros_driver_lib.dir/src/accel_epsonA352.c.o
[ 77%] Linking C shared library /home/guest/catkin_ws/devel/lib/libepson_accl_ros_driver_lib.so
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[ 77%] Built target epson_accl_ros_driver_lib
Scanning dependencies of target epson_accl_ros_node
make[2]: Warning: File '/home/guest/catkin_ws/src/accl_ros_node/src/epson_accl_uart_node.cpp' has modification time 3785910 s in the future
[ 88%] Building CXX object accl_ros_node/CMakeFiles/epson_accl_ros_node.dir/src/epson_accl_uart_node.cpp.o
In file included from /usr/include/boost/bind.hpp:22:0,
                 from /opt/ros/melodic/include/ros/publisher.h:35,
                 from /opt/ros/melodic/include/ros/node_handle.h:32,
                 from /opt/ros/melodic/include/ros/ros.h:45,
                 from /home/guest/catkin_ws/src/accl_ros_node/src/epson_accl_uart_node.cpp:50:
/usr/include/boost/bind/bind_cc.hpp: In function `boost::_bi::bind_t<R, R (*)(B1), typename boost::_bi::list_av_1<A1>::type> boost::bind(R (*)(B1), A1) [with R = ros::SerializedMessage; B1 = const sensor_msgs::Imu_<std::allocator<void> >&; A1 = boost::reference_wrapper<const sensor_msgs::Imu_<std::allocator<void> > >]':
/usr/include/boost/bind/bind_cc.hpp:26:5: note: parameter passing for argument of type `boost::reference_wrapper<const sensor_msgs::Imu_<std::allocator<void> > >' changed in GCC 7.1
     BOOST_BIND(BOOST_BIND_ST R (BOOST_BIND_CC *f) (B1), A1 a1)
     ^
In file included from /usr/include/boost/bind/bind.hpp:2126:0,
                 from /usr/include/boost/bind.hpp:22,
                 from /opt/ros/melodic/include/ros/publisher.h:35,
                 from /opt/ros/melodic/include/ros/node_handle.h:32,
                 from /opt/ros/melodic/include/ros/ros.h:45,
                 from /home/guest/catkin_ws/src/accl_ros_node/src/epson_accl_uart_node.cpp:50:
/usr/include/boost/bind/bind_cc.hpp:30:58: note: parameter passing for argument of type `boost::reference_wrapper<const sensor_msgs::Imu_<std::allocator<void> > >' changed in GCC 7.1
     return _bi::bind_t<R, F, list_type> (f, list_type(a1));
                                                          ^
In file included from /usr/include/boost/bind.hpp:22:0,
                 from /opt/ros/melodic/include/ros/publisher.h:35,
                 from /opt/ros/melodic/include/ros/node_handle.h:32,
                 from /opt/ros/melodic/include/ros/ros.h:45,
                 from /home/guest/catkin_ws/src/accl_ros_node/src/epson_accl_uart_node.cpp:50:
/usr/include/boost/bind/bind.hpp: In constructor `boost::_bi::list1<A1>::list1(A1) [with A1 = boost::reference_wrapper<const sensor_msgs::Imu_<std::allocator<void> > >]':
/usr/include/boost/bind/bind.hpp:231:14: note: parameter passing for argument of type `boost::reference_wrapper<const sensor_msgs::Imu_<std::allocator<void> > >' changed in GCC 7.1
     explicit list1( A1 a1 ): base_type( a1 ) {}
              ^~~~~
/usr/include/boost/bind/bind.hpp:231:44: note: parameter passing for argument of type `boost::reference_wrapper<const sensor_msgs::Imu_<std::allocator<void> > >' changed in GCC 7.1
     explicit list1( A1 a1 ): base_type( a1 ) {}
                                            ^
In file included from /usr/include/boost/bind/bind.hpp:47:0,
                 from /usr/include/boost/bind.hpp:22,
                 from /opt/ros/melodic/include/ros/publisher.h:35,
                 from /opt/ros/melodic/include/ros/node_handle.h:32,
                 from /opt/ros/melodic/include/ros/ros.h:45,
                 from /home/guest/catkin_ws/src/accl_ros_node/src/epson_accl_uart_node.cpp:50:
/usr/include/boost/bind/storage.hpp: In constructor `boost::_bi::storage1<A1>::storage1(A1) [with A1 = boost::reference_wrapper<const sensor_msgs::Imu_<std::allocator<void> > >]':
/usr/include/boost/bind/storage.hpp:42:14: note: parameter passing for argument of type `boost::reference_wrapper<const sensor_msgs::Imu_<std::allocator<void> > >' changed in GCC 7.1
     explicit storage1( A1 a1 ): a1_( a1 ) {}
              ^~~~~~~~
[100%] Linking CXX executable /home/guest/catkin_ws/devel/lib/epson_accl_ros_driver/epson_accl_ros_node
make[2]: warning:  Clock skew detected.  Your build may be incomplete.
[100%] Built target epson_accl_ros_node
guest@guest-desktop:~/catkin_ws$

```

4. Reload the current ROS environment variables that may have changed after the catkin build process.
```
   From the <catkin_workspace>: source devel/setup.bash
```

5. Modify the appropriate launch file for the ACCL model in the launch/ folder to set your desired ACCL configure parameter options at runtime:
**NOTE:** Refer to the ROS1 launch file for inline descriptions.

Parameter            | Comment
-------------------- | -------------
port                 | specifies the string value to the tty serial device (ignored for SPI)
mesmod_sel           | specifies standard noise floor or reduced noise floor mode
temp_stabil          | specifies to enable or disable temperature stabilization
ext_sel              | specifies to enable or disable External Trigger
ext_pol              | specifies the polarity of the External Trigger
drdy_on              | specifies to enable or disable DRDY function
drdy_pol             | specifies the polarity of the DRDY input pin when enabled
dout_rate            | specifies the ACCL output data rate
filter_sel           | specifies the ACCL filter setting
flag_out             | specifies to enable or disable ND_FLAG status in ACCL output data (not used by ROS)
temp_out             | specifies to enable or disable TempC sensor in ACCL output data (not used by ROS)
accel_out            | specifies to enable or disable Accl sensor in ACCL output data (must be enabled)
count_out            | specifies to enable or disable counter in ACCL output data
checksum_out         | specifies to enable or disable checksum in ACCL output data (when enabled checksum errors are detected)

**NOTE:** The ROS1 launch file passes ACCL configuration settings to the ACCL at runtime.
           Therefore does not need rebuilding with catkin when changing the launch file.

6. To start the Epson ACCL ROS1 driver use the appropriate launch file (located in launch/) from console.
- The launch file contains parameters for configuring settings at runtime
- All parameters are described in the inline comments of the launch file.

   For example, for the Epson A352 ACCL:
```
   <catkin_workspace>/roslaunch epson_accl_ros_driver epson_a352.launch
```


### Example console output of launching ROS1 node for A352 UART:
```
guest@guest-desktop:~/catkin_ws$ roslaunch epson_accl_ros_driver epson_a352.launch
... logging to /home/guest/.ros/log/b5e1b5be-17e5-11eb-ab08-b827eba23167/roslaunch-guest-desktop-1882.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://guest-desktop:36717/

SUMMARY
========

PARAMETERS
 * /epson_accl_ros_node/accel_out: 1
 * /epson_accl_ros_node/checksum_out: 1
 * /epson_accl_ros_node/count_out: 1
 * /epson_accl_ros_node/dout_rate: 4
 * /epson_accl_ros_node/drdy_on: 1
 * /epson_accl_ros_node/drdy_pol: 1
 * /epson_accl_ros_node/ext_pol: 0
 * /epson_accl_ros_node/ext_sel: 0
 * /epson_accl_ros_node/filter_sel: 8
 * /epson_accl_ros_node/flag_out: 1
 * /epson_accl_ros_node/mesmod_sel: 0
 * /epson_accl_ros_node/port: /dev/ttyUSB0
 * /epson_accl_ros_node/temp_out: 1
 * /epson_accl_ros_node/temp_stabil: 0
 * /rosdistro: melodic
 * /rosversion: 1.14.9

NODES
  /
    epson_accl_ros_node (epson_accl_ros_driver/epson_accl_ros_node)

auto-starting new master
process[master]: started with pid [1893]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to b5e1b5be-17e5-11eb-ab08-b827eba23167
process[rosout-1]: started with pid [1904]
started core service [/rosout]
process[epson_accl_ros_node-2]: started with pid [1907]
[ INFO] [1603756104.835468494]: Initializing HCL layer...
[ INFO] [1603756104.841612782]: Initializing GPIO interface...
[ INFO] [1603756104.841852260]: Initializing UART interface...
Attempting to open port.../dev/ttyUSB0
[ INFO] [1603756104.844636150]: Checking sensor NOT_READY status...
...done.[ INFO] [1603756105.802907816]: Initializing Sensor...
[ INFO] [1603756105.820638757]: Epson ACCL initialized.
[ INFO] [1603756105.883478867]: PRODUCT ID:     A352AD10
[ INFO] [1603756105.947734021]: SERIAL ID:      W0000501
 
```


## What does this ROS1 ACCL node publish as an Messages?

- The Epson Accelerometer ROS1 driver will publish messages which will vary slightly on the output configuration.
- For ACCL models such as A352, the ACCL messages will only contain fields for linear acceleration (accel) data the other fields will contain 0s

The Epson Accelerometer ROS1 driver will publish ACCL messages as convention per [REP 145](http://www.ros.org/reps/rep-0145.html).

### Example ROS1 topic output
The ROS1 driver will publish to the following ROS topics:
```
/epson_accl/data_raw <-- orientation, angular_velocity, & covariance fields will not contain valid data & should be ignored
```
**NOTE** The launch file will remap the message to publish on /imu/data_raw


#### ROS Topic Message /imu/data_raw
```
---
header:
  seq: 18833
  stamp:
    secs: 1603756210
    nsecs: 518862006
  frame_id: "imu_link"
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0
orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: 0.0
  y: 0.0
  z: 0.0
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: 2.56680425218
  y: -3.00368994106
  z: 9.0503388805
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---

```


## Why am I seeing high latencies or slower than expected ACCL data rates

### For SPI interface
- This will largely depend on your host system processing load and latency.
- If your ROS platform is running too many ROS node packages or simply too slow it may not be able detect the rising edge of the ACCL DRDY signal and then burst read the ACCL sampling data and post process it.
- Try modifying the *dout_rate* and *filter_sel* to the slowest setting that
can meet your ROS system requirements. 
- Monitoring the DRDY signal on the ACCL to verify the stability of the ACCL
DRDY signal is recommended when experiencing data rate issues.

### For uART interface
- If your connection between the Epson Accelerometer UART and the Linux host is by FTDI (or similar USB-UART bridge devices, the default latency_timer setting may be too large (i.e. typically 16msec).
- There are 2 recommended methods to reduce this value to 1msec.

#### Modifying latency_timer by sysfs mechanism

- The example below reads the latency_timer setting for /dev/ttyUSB0 which returns 16msec.
- Then, the latency_timer is set to 1msec, and confirmed by readback.

*NOTE*: May require root (sudo su) access on your system to modify. 
```
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
16
echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
1
```

#### Modifying low_latency flag using setserial utility

The example below sets the low_latency flag for /dev/ttyUSB0.
This will have the same effect as setting the latency_timer to 1msec.
This can be confirmed by running the setserial command again.

```
user@user-VirtualBox:~$ setserial /dev/ttyUSB0
/dev/ttyUSB0, UART: unknown, Port: 0x0000, IRQ: 0

user@user-VirtualBox:~$ setserial /dev/ttyUSB0 low_latency

user@user-VirtualBox:~$ setserial /dev/ttyUSB0
/dev/ttyUSB0, UART: unknown, Port: 0x0000, IRQ: 0, Flags: low_latency
```


## Package Contents

The Epson Accelerometer ROS1 driver-related sub-folders & root files are:
```
   launch/        <== example launch file for Epson Accelerometer models
   src/           <== source code for ROS node C++ wrapper, ACCL C driver, and additional README_src.md
   CmakeLists.txt <== build script for catkin_make
   package.xml    <== catkin package description
   README.md      <== general README for the ROS1 driver
```


## License

### The Epson ACCL C++ Wrapper ROS1 Node is released under BSD-3 license.

[This software is BSD-3 licensed.](http://opensource.org/licenses/BSD-3-Clause)

Copyright (c) 2020, 2021 Seiko Epson Corp. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

### The Epson Accelerometer C driver software is released as public domain.

THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
NONINFRINGEMENT, SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A
PARTICULAR PURPOSE. IN NO EVENT SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE
OR CLAIM, ARISING FROM OR IN CONNECTION WITH THE SOFTWARE OR THE USE OF THE
SOFTWARE.


## References
1. https://index.ros.org
