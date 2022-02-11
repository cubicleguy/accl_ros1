//==============================================================================
//
// 	epson_accl_spi_node.cpp
//     - ROS node for Epson ACCL sensor evaluation
//     - This program initializes the Epson IMU and publishes ROS messages in
//       ROS topic /epson_accl as convention per [REP 145]
//       (http://www.ros.org/reps/rep-0145.html).
//     - The Orientation or Gyroscope fields do not update in published
//       topic /epson_accl/data_raw
//
//  [This software is BSD-3
//  licensed.](http://opensource.org/licenses/BSD-3-Clause)
//
//  Copyright (c) 2020, 2021, Seiko Epson Corp. All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
//  THE POSSIBILITY OF SUCH DAMAGE.
//
//==============================================================================

extern "C" {
#include "accel_epsonCommon.h"
#include "hcl.h"
#include "hcl_gpio.h"
#include "hcl_spi.h"
}

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <iostream>
#include <string>

using namespace std;

//=========================================================================
//------------------------ IMU Initialization -----------------------------
//=========================================================================

bool init(const struct EpsonAcclOptions& options) {
  ROS_INFO("Initializing HCL layer...");
  if (!seInit()) {
    ROS_ERROR(
        "Error: could not initialize the Seiko Epson HCL layer. Exiting...");
    return false;
  }

  ROS_INFO("Initializing GPIO interface...");
  if (!gpioInit()) {
    ROS_ERROR("Error: could not initialize the GPIO layer. Exiting...");
    seRelease();
    return false;
  }

  ROS_INFO("Initializing SPI interface...");
  // The max SPI clock rate is 1MHz for current model Epson IMUs
  if (!spiInit(SPI_MODE3, 500000)) {
    ROS_ERROR("Error: could not initialize SPI interface. Exiting...");
    gpioRelease();
    seRelease();
    return false;
  }

  ROS_INFO("Checking sensor NOT_READY status...");
  if (!acclPowerOn()) {
    ROS_ERROR("Error: failed to power on Sensor. Exiting...");
    spiRelease();
    gpioRelease();
    seRelease();
    return false;
  }
  printf("...done.");

  ROS_INFO("Initializing Sensor...");
  if (!acclInitOptions(options)) {
    ROS_ERROR("Error: could not initialize Epson Sensor. Exiting...");
    spiRelease();
    gpioRelease();
    seRelease();
    return false;
  }

  ROS_INFO("Epson ACCL initialized.");
  return true;
}


//=========================================================================
//------------------------ IMU Identify PROD_ID & SER_NUM -----------------
//=========================================================================

std::string get_prod_id() {
  unsigned short prod_id1 = registerRead16(CMD_WINDOW1, ADDR_PROD_ID1, false);
  unsigned short prod_id2 = registerRead16(CMD_WINDOW1, ADDR_PROD_ID2, false);
  unsigned short prod_id3 = registerRead16(CMD_WINDOW1, ADDR_PROD_ID3, false);
  unsigned short prod_id4 = registerRead16(CMD_WINDOW1, ADDR_PROD_ID4, false);

  char myarray[] = {
	static_cast<char>(prod_id1), static_cast<char>(prod_id1 >> 8),
	static_cast<char>(prod_id2), static_cast<char>(prod_id2 >> 8),
	static_cast<char>(prod_id3), static_cast<char>(prod_id3 >> 8),
	static_cast<char>(prod_id4), static_cast<char>(prod_id4 >> 8)};
  std::string prod_id(myarray);
  return prod_id;
}

std::string get_serial_id() {
  unsigned short ser_num1 =
	registerRead16(CMD_WINDOW1, ADDR_SERIAL_NUM1, false);
  unsigned short ser_num2 =
	registerRead16(CMD_WINDOW1, ADDR_SERIAL_NUM2, false);
  unsigned short ser_num3 =
	registerRead16(CMD_WINDOW1, ADDR_SERIAL_NUM3, false);
  unsigned short ser_num4 =
	registerRead16(CMD_WINDOW1, ADDR_SERIAL_NUM4, false);

  char myarray[] = {
	static_cast<char>(ser_num1), static_cast<char>(ser_num1 >> 8),
	static_cast<char>(ser_num2), static_cast<char>(ser_num2 >> 8),
	static_cast<char>(ser_num3), static_cast<char>(ser_num3 >> 8),
	static_cast<char>(ser_num4), static_cast<char>(ser_num4 >> 8)};
  std::string ser_num(myarray);
  return ser_num;
}

void identify_device() {
  ROS_INFO("PRODUCT ID:\t%s", get_prod_id().c_str());
  ROS_INFO("SERIAL ID:\t%s", get_serial_id().c_str());
}

//=========================================================================
//------------------------------ Main -------------------------------------
//=========================================================================

int main(int argc, char** argv) {
  ros::init(argc, argv, "epson_accl_spi_node");
  ros::NodeHandle nh;
  ros::NodeHandle np("~");

  const double GRAVITY = 9.80665;
  struct EpsonAcclOptions options={0};

  // Recommended to change these parameters via .launch file instead of
  // modifying source code below directly
  np.param("mesmod_sel", options.mesmod_sel, 0);
  np.param("temp_stabil", options.temp_stabil, 0);
  np.param("ext_sel", options.ext_sel, 0);
  np.param("ext_pol", options.ext_pol, 0);
  np.param("drdy_on", options.drdy_on, 1);
  np.param("drdy_pol", options.drdy_pol, 1);

  np.param("dout_rate", options.dout_rate, CMD_RATE200);
  np.param("filter_sel", options.filter_sel, CMD_FIRTAP512FC16);

  np.param("flag_out", options.flag_out, 1);
  np.param("temp_out", options.temp_out, 1);
  np.param("accel_out", options.accel_out, 1);
  np.param("count_out", options.count_out, 1);
  np.param("checksum_out", options.checksum_out, 1);

  if (!init(options)) {
    ROS_ERROR(
        "Error: failed on init(). Exiting...");
    return -1;
  }
  identify_device();
  acclStart();

  struct EpsonAcclData epson_data = {0};

  sensor_msgs::Imu imu_msg;
  for (int i = 0; i < 9; i++) {
    imu_msg.orientation_covariance[i] = 0;
    imu_msg.angular_velocity_covariance[i] = 0;
    imu_msg.linear_acceleration_covariance[i] = 0;
  }
  imu_msg.orientation_covariance[0] = -1;

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("epson_accl", 1);

  while (ros::ok()) {
    if (acclDataReady()) {
      if (acclDataReadBurstNOptions(options, &epson_data)) {
        imu_msg.header.frame_id = "imu_link";
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.linear_acceleration.x = epson_data.accel_x * GRAVITY;
        imu_msg.linear_acceleration.y = epson_data.accel_y * GRAVITY;
        imu_msg.linear_acceleration.z = epson_data.accel_z * GRAVITY;
        imu_pub.publish(imu_msg);
      }
    }
  }

  acclStop();
  seDelayMS(1000);
  spiRelease();
  gpioRelease();
  seRelease();

  return 0;
}
