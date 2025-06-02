// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#ifndef LEUZE_RSL_DRIVER__INTERFACE_HPP_
#define LEUZE_RSL_DRIVER__INTERFACE_HPP_

#include <angles/angles.h>
#include <algorithm>
#include <limits>
#include <vector>
#include <string>
#include "leuze_rsl_driver/communication.hpp"
#include "leuze_rsl_driver/hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


using LaserScan = sensor_msgs::msg::LaserScan;
using String = std_msgs::msg::String;


template<typename ExtendedStatusProfileMsg>
class RslInterface : public rclcpp::Node, HardwareInterface<UDPConnection>, DataParser
{
public:
  RslInterface(std::string address, std::string port)
  : Node("leuze_driver"), HardwareInterface(address, port, this)
  {
    // No action needed
  }
  ~RslInterface()
  {
    disconnect();
  }

  void connect()
  {
    HardwareInterface<UDPConnection>::connect();
    RCLCPP_INFO_STREAM(get_logger(), "[Laser Scanner] Listening to data");
  }
  void disconnect()
  {
    HardwareInterface<UDPConnection>::disconnect();
    RCLCPP_INFO_STREAM(get_logger(), "[Laser Scanner] RSL Disconnected");
  }

  int parseBuffer(std::basic_string<unsigned char> buffer)
  {
    if (debug_on) {
      LogBufferToDebug(buffer);
    }

    Frame * frame = reinterpret_cast<Frame *>(const_cast<unsigned char *>(buffer.c_str()) );

    // To get the scanner type (0 for RSL400, 1 for RSL200) extract the highest 8 bits of
    // the received frame ID. This is commented out because we do not use automatic device
    // type detection in this driver:
    // uint8_t scanner_type = static_cast<uint8_t>(frame->id >> 8);

    // From now on, we only use the remaining (lowest) 8 bits for frame ID:
    frame->id = static_cast<uint8_t>(frame->id);

    if (frame->id == 1) {
      // Extended status profile. Status profile + measurement contour descritpion. Pg8 3.3.1.
      parseExtendedStatusProfile(buffer);
    } else if ( (frame->id == 3) || (frame->id == 6) ) {
      if (configuration_received_ == false) {
        RCLCPP_INFO(
          get_logger(),
          "[Laser Scanner] Scan data header not received, skipping measurement.");
      } else {
        if (static_cast<int>(frame->scan_number) != scan_number_) {
          RCLCPP_INFO(
            get_logger(),
            "[Laser Scanner] Unexpected Scan Data id, skipping measurement.");
        } else {
          scan_data_[frame->block] = parseScanData(buffer, frame);
        }
      }
    } else if (frame->id == 0) {
      return frame->id;
    } else {
      RCLCPP_INFO(get_logger(), "[Laser Scanner] Unknown frame type ID : %d", frame->id);
      return -1;
    }

    if (measure_counter_ == scan_size_) {
      if (checkScan()) {
        publishScan();
      }
    }

    if (measure_counter_ >= scan_size_) {
      RCLCPP_WARN(get_logger(), "[Laser Scanner] Scan measure counter overflowed, resetting");
      configuration_received_ = false;
      resetDefault();
    }

    return frame->id;
  }

protected:
  virtual void resetDefault() = 0;
  bool checkScan()
  {
    int i_measure = 0;
    // Assemble data from block;
    for (int i_block = 0; i_block < block_counter_; i_block++) {
      if (scan_data_[i_block].data_distance.size() == 0) {
        RCLCPP_INFO(
          get_logger(),
          "[Laser Scanner] Received scan data datagram with no distance values");
        return false;
      } else {
        for (unsigned int i_scan = 0; i_scan < scan_data_[i_block].data_distance.size(); i_scan++) {
          laser_scan_.ranges[i_measure] =
            static_cast<float>(scan_data_[i_block].data_distance[i_scan]) / 1000.0;
          // std::cout << laser_scan_.ranges.size() << " ";
          laser_scan_.intensities[i_measure] =
            static_cast<float>(scan_data_[i_block].data_signal_strength[i_scan]);
          i_measure++;
        }
        laser_scan_.ranges[scan_size_] = std::numeric_limits<double>::infinity();
        // std::cout << std::endl;
      }
    }

    laser_scan_.header.stamp = this->get_clock()->now();
    // Reverse the scans to match real world
    std::reverse(laser_scan_.ranges.begin(), laser_scan_.ranges.end());
    std::reverse(laser_scan_.intensities.begin(), laser_scan_.intensities.end());
    return true;
  }


  DatagramMeasurementDataType parseScanData(std::basic_string<unsigned char> buffer, Frame * frame)
  {
    DatagramMeasurementDataType mdt;
    unsigned int length = 0;
    mdt.frame = frame;
    if (frame->id == 3) {
      length = (frame->h1.total_length - 20) / 4;

      // Capturing 4 bytes at a time - 2 for distance and 2 for signal strength
      // as per UDP spec pg 14 table 3.6
      for (unsigned int i = 20; i < buffer.length(); i += 4) {
        uint16_t distance = convertBytesToUint16(buffer[i], buffer[i + 1]);
        uint16_t intensity = convertBytesToUint16(buffer[i + 2], buffer[i + 3]);
        mdt.data_distance.push_back(distance);
        mdt.data_signal_strength.push_back(intensity);
      }
    } else if (frame->id == 6) {
      length = (frame->h1.total_length - 20) / 2;
      // Capturing 2 bytes at a time for distance
      for (unsigned int i = 20; i < buffer.length(); i += 2) {
        uint16_t distance = convertBytesToUint16(buffer[i], buffer[i + 1]);
        mdt.data_distance.push_back(distance);
        mdt.data_signal_strength.push_back(0.0);
      }
    }

    // Buffer length should match length declared by header
    if (mdt.frame->h1.total_length != buffer.length()) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "[Laser Scanner] Parsing Scan data message of incorrect length "
          << buffer.length() << ", expected " << mdt.frame->h1.total_length);
      return mdt;
    }
    // Number of distance/signal values is equal to data length in bytes/4
    // (because 4 bytes per value)
    // Data langth is total length of datagram - 20 (which is fixed frame size)
    // Refer UDP specs pg 14 3.3.2.2. This gives number of "measurement data values"/"scan values".
    if (mdt.data_distance.size() != length) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "[Laser Scanner] Parsing Scan data message of incorrect number of data values "
          << mdt.data_distance.size() << ", expected " << length);
      return mdt;
    }
    if (mdt.data_signal_strength.size() != length) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "[Laser Scanner] Parsing Scan data message of incorrect number of signal strength values "
          << mdt.data_signal_strength.size() << ", expected " << length);
      return mdt;
    }

    measure_counter_ += length;
    block_counter_ = mdt.frame->block + 1;
    return mdt;
  }


  uint16_t convertBytesToUint16(unsigned char low_byte, unsigned char high_byte)
  {
    return high_byte << 8 | low_byte;
  }


  bool compareTwoFloats(float a, float b, float epsilon = 0.0001)
  {
    return fabs(a - b) < epsilon;
  }


  virtual void parseExtendedStatusProfile(std::basic_string<unsigned char> buffer) = 0;
  rclcpp::Publisher<LaserScan>::SharedPtr pub_scan_;
  typename rclcpp::Publisher<ExtendedStatusProfileMsg>::SharedPtr pub_status_;
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


  void LogBufferToDebug(std::basic_string<unsigned char> buffer)
  {
    std::stringstream oss;
    oss << std::hex;
    for (unsigned int i = 0; i < buffer.length(); i++) {
      oss << std::setw(2) << std::setfill('0') << static_cast<u_int16_t>(buffer[i]);
    }

    String string_msg;
    string_msg.data = oss.str();
    pub_debug_->publish(string_msg);
  }


  void publishScan()
  {
    pub_scan_->publish(laser_scan_);
    pub_status_->publish(status_msg_);
    measure_counter_ = 0;
  }
};

#endif  // LEUZE_RSL_DRIVER__INTERFACE_HPP_
