cmake_minimum_required(VERSION 2.8.3)
project(epson_accl_ros_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Specify compiler options
set(CMAKE_CXX_COMPILER "/usr/bin/g++")

# Default to C99
if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
      set (CMAKE_C_FLAGS "-std=gnu99 ${CMAKE_C_FLAGS}")
    endif ()
else ()
    set (CMAKE_C_STANDARD 99)
endif ()

# Specify DEBUG macro to enable any debug code by adding "-DDEBUG" in add_definitions()
#add_definitions(-DDEBUG)

# Refer to the README inside the src folder for more details
# Currently only A352 is supported at the time of writing
set(accl_model "A352")
add_definitions(-D${accl_model})
message("---- Building for ACCL Model: ${accl_model}")

# Select Serial Interface Type to UART or SPI
set(interface "UART")
#set(interface "SPI")
add_definitions(-D${interface})
message("---- Building for interface: ${interface}")

# Select Platform macro depending on IF type (SPI requires RaspberryPi)
if (interface STREQUAL "SPI")
  set(platform "RPI")
  add_definitions(-D${platform})
elseif (interface STREQUAL "UART")
  set(platform "NONE")
else()
  message([FATAL_ERROR] "**** Invalid interface type")
endif()
message("---- Building for platform: ${platform}")


# Create filelist of C libraries for SPI 
if (interface STREQUAL "SPI")
  set(lib_sources 
    src/hcl_rpi.c
    src/hcl_gpio_rpi.c
    src/hcl_spi_rpi.c
    src/accel_epsonCommon.c
    src/accel_epsonSpi.c
  )
elseif (interface STREQUAL "UART")
# Create filelist of C libraries for UART
  set(lib_sources 
    src/hcl_linux.c
    src/hcl_gpio.c
    src/hcl_uart.c
    src/accel_epsonCommon.c
    src/accel_epsonUart.c
  )
endif()


# Add ACCL model specific source to library 
if (accl_model STREQUAL "A352")
  set(lib_sources ${lib_sources}
    src/accel_epsonA352.c)
else()
  message([FATAL_ERROR] "**** Invalid ACCL Model")
endif()

# Declare a library for Epson ACCL functions from C sources
add_library(epson_accl_ros_driver_lib
  ${lib_sources}
)


# Determine location of wiringPi library on the host system
# Needed if building on Raspberry Pi platform
if (interface STREQUAL "SPI")
  find_library(wiringPi_LIB NAMES wiringPi)
  target_link_libraries(epson_accl_ros_driver_lib 
    ${epson_accl_ros_driver_lib}
    ${wiringPi_LIB}
  )
endif()


# Declare a C++ executable
if (interface STREQUAL "SPI")
  add_executable(epson_accl_ros_node src/epson_accl_spi_node.cpp)
else()
  add_executable(epson_accl_ros_node src/epson_accl_uart_node.cpp)
endif()

# Link epson C library to executable target ROS node
target_link_libraries(epson_accl_ros_node
 ${catkin_LIBRARIES}
 epson_accl_ros_driver_lib
)

# Debugging
#message(INFO "${catkin_LIBRARIES}")
#message(INFO "${catkin_INCLUDE_DIRS}")
#message(INFO "${wiringPi_LIB}")
#message(INFO "${lib_sources}")
#message(INFO "${epson_accl_ros_driver_lib}")

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS epson_accl_ros_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

