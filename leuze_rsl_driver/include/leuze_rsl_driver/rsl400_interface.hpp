// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#ifndef LEUZE_RSL_DRIVER__RSL400_INTERFACE_HPP_
#define LEUZE_RSL_DRIVER__RSL400_INTERFACE_HPP_

#include <angles/angles.h>
#include <algorithm>
#include <vector>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "leuze_rsl_driver/communication.hpp"
#include "leuze_rsl_driver/hardware_interface.hpp"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "leuze_msgs/msg/extended_status_profile_msg_rsl400.hpp"

using LaserScan = sensor_msgs::msg::LaserScan;
using ExtendedStatusProfileMsg = leuze_msgs::msg::ExtendedStatusProfileMsgRsl400;
using String = std_msgs::msg::String;


class RSL400Interface : public rclcpp::Node, HardwareInterface<UDPConnection>, DataParser
{
public:
  RSL400Interface(std::string address, std::string port, std::string topic);
  ~RSL400Interface();

  void connect();
  void disconnect();
  int parseBuffer(std::basic_string<unsigned char> buffer);

protected:
  void resetDefault();
  bool checkScan();
  void publishScan();
  void verifyConfiguration(DatagramExtendedStatusProfile_rsl400 d_esp);
  DatagramExtendedStatusProfile_rsl400 parseExtendedStatusProfile(
    std::basic_string<unsigned char> buffer);
  DatagramMeasurementDataType parseScanData(
    std::basic_string<unsigned char> buffer, Frame * frame);
  uint16_t convertBytesToUint16(unsigned char low_byte, unsigned char high_byte);
  bool compareTwoFloats(float a, float b, float epsilon = 0.0001);

private:
  rclcpp::Publisher<LaserScan>::SharedPtr pub_scan_;
  rclcpp::Publisher<ExtendedStatusProfileMsg>::SharedPtr pub_status_;
  rclcpp::Publisher<String>::SharedPtr pub_debug_;

  std::string header_frame_;
  bool configuration_received_;
  bool debug_on;

  LaserScan laser_scan_;
  ExtendedStatusProfileMsg status_msg_;

  int scan_number_;
  int configuration_type_;  // type 3 = Distance + Intensity / type6 = Distance
  int measure_counter_;
  int block_counter_;
  int scan_size_;

  std::vector<DatagramMeasurementDataType> scan_data_;

  void LogBufferToDebug(std::basic_string<unsigned char> buffer);

  // HTTP protocol object
};


#endif  // LEUZE_RSL_DRIVER__RSL400_INTERFACE_HPP_
