// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#include "leuze_rsl_driver/rsl400_interface.hpp"
#include <angles/angles.h>
#include <algorithm>


RSL400Interface::RSL400Interface(std::string address, std::string port, std::string topic)
: RslInterface(address, port)
{
  // angle range -135° to +135°, step 0.1
  pub_scan_ = this->create_publisher<LaserScan>(topic, 50);
  pub_status_ = this->create_publisher
    <leuze_msgs::msg::ExtendedStatusProfileMsgRsl400>("status", 50);
  configuration_received_ = false;

  // Get param scan_frame
  this->declare_parameter("scan_frame", rclcpp::PARAMETER_STRING);
  std::string scan_frame;
  try {
    scan_frame = this->get_parameter("scan_frame").as_string();
  } catch (const std::exception & e) {
    scan_frame = "scanner_laser";  // Default value
  }

  // Get param ls_debug
  this->declare_parameter("ls_debug", "false");
  auto ls_debug = this->get_parameter("ls_debug").as_string() == "true" ? true : false;

  // Get param scan_size
  this->declare_parameter("scan_size", rclcpp::PARAMETER_INTEGER);
  try {
    scan_size_ = this->get_parameter("scan_size").as_int();
  } catch (const std::exception & e) {
    scan_size_ = 2700;  // Default value
  }

  scan_data_.resize(scan_size_);

  RCLCPP_INFO(get_logger(), "scan_frame: %s", scan_frame.c_str());
  RCLCPP_INFO(get_logger(), "ls_debug: %s", ls_debug == true ? "true" : "false");

  header_frame_ = scan_frame;

  if (ls_debug == true) {
    debug_on = true;
  }

  if (debug_on) {
    pub_debug_ = this->create_publisher<String>("scan_raw_data", 50);
    RCLCPP_WARN(
      get_logger(),
      "[Laser Scanner] Debug Mode is on. This might affect the performace.");
  }
  resetDefault();
}

RSL400Interface::~RSL400Interface()
{
  disconnect();
}


void RSL400Interface::resetDefault()
{
  this->declare_parameter("angle_min", rclcpp::PARAMETER_DOUBLE);  // Default min val (-135°)
  this->declare_parameter("angle_max", rclcpp::PARAMETER_DOUBLE);  // Default min value (+135°)
  this->declare_parameter("scan_time", rclcpp::PARAMETER_DOUBLE);  // 0.04; Default
  this->declare_parameter("range_min", rclcpp::PARAMETER_DOUBLE);  // 0.001; Default
  this->declare_parameter("range_max", rclcpp::PARAMETER_DOUBLE);  // 65.0; Max range 65m

  double angle_min;
  try {
    angle_min = this->get_parameter("angle_min").as_double();
  } catch (const std::exception & e) {
    angle_min = -2.35619449;  // Default value
  }

  double angle_max;
  try {
    angle_max = this->get_parameter("angle_max").as_double();
  } catch (const std::exception & e) {
    angle_max = 2.35619449;  // Default value
  }

  double scan_time;
  try {
    scan_time = this->get_parameter("scan_time").as_double();
  } catch (const std::exception & e) {
    scan_time = 0.04;  // Default value
  }

  double range_min;
  try {
    range_min = this->get_parameter("range_min").as_double();
  } catch (const std::exception & e) {
    range_min = 0.001;  // Default value
  }

  double range_max;
  try {
    range_max = this->get_parameter("range_max").as_double();
  } catch (const std::exception & e) {
    range_max = 65.0;  // Default value
  }

  RCLCPP_INFO(get_logger(), "angle_min: %f", angle_min);
  RCLCPP_INFO(get_logger(), "angle_max: %f", angle_max);
  RCLCPP_INFO(get_logger(), "scan_time: %f", scan_time);
  RCLCPP_INFO(get_logger(), "range_min: %f", range_min);
  RCLCPP_INFO(get_logger(), "range_max: %f", range_max);

  laser_scan_.header.frame_id = header_frame_;
  laser_scan_.angle_min = angle_min;  // Default min value
  laser_scan_.angle_max = angle_max;  // Default min value
  // Default max resolution:
  laser_scan_.angle_increment = (laser_scan_.angle_max - laser_scan_.angle_min) /
    static_cast<float>(scan_size_);
  laser_scan_.scan_time = scan_time;  // Default
  laser_scan_.range_min = range_min;  // Default
  laser_scan_.range_max = range_max;  // Max range 65m

  laser_scan_.ranges.resize(0);
  laser_scan_.ranges.resize(scan_size_ + 1);
  laser_scan_.intensities.resize(0);
  laser_scan_.intensities.resize(scan_size_ + 1);

  block_counter_ = 0;
  measure_counter_ = 0;
  scan_number_ = -1;  // Get last scan number

  RCLCPP_INFO_STREAM(get_logger(), "[Laser Scanner] Reset data");
}


void RSL400Interface::parseExtendedStatusProfile(
  std::basic_string<unsigned char> buffer)
{
  DatagramExtendedStatusProfile_rsl400 * esp =
    reinterpret_cast<DatagramExtendedStatusProfile_rsl400 *>(
    const_cast<unsigned char *>(buffer.c_str())
    );

  if (buffer.length() != esp->frame.h1.total_length) {
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "[Laser Scanner] Parsing Extended Status Profile of incorrect length "
        << buffer.length() << ", expected " << esp->frame.h1.total_length);
  }
  verifyConfiguration(*esp);

  status_msg_.header.frame_id = header_frame_;
  status_msg_.header.stamp = this->get_clock()->now();

  status_msg_.byte_0 = esp->status_profile.byte_0;
  status_msg_.byte_1 = esp->status_profile.byte_1;
  status_msg_.msg_and_ossd_2 = esp->status_profile.msg_and_ossd_2;
  status_msg_.emergency_stop_3 = esp->status_profile.emergency_stop_3;
  status_msg_.electrical_signals_byte_4 = esp->status_profile.electrical_signals_byte_4;
  status_msg_.electrical_signals_byte_5 = esp->status_profile.electrical_signals_byte_5;
  status_msg_.electrical_signals_byte_6 = esp->status_profile.electrical_signals_byte_6;
  status_msg_.electrical_signals_byte_7 = esp->status_profile.electrical_signals_byte_7;
  status_msg_.scan_number = esp->status_profile.scan_number;
  status_msg_.protec_func_a_12 = esp->status_profile.protec_func_a_12;
  status_msg_.fp_sel_a_byte_13 = esp->status_profile.fp_sel_a_byte_13;
  status_msg_.fp_sel_a_byte_14 = esp->status_profile.fp_sel_a_byte_14;
  status_msg_.indic_a_15 = esp->status_profile.indic_a_15;
  status_msg_.protec_func_b_16 = esp->status_profile.protec_func_b_16;
  status_msg_.fp_sel_b_byte_17 = esp->status_profile.fp_sel_b_byte_17;
  status_msg_.fp_sel_b_byte_18 = esp->status_profile.fp_sel_b_byte_18;
  status_msg_.indic_b_19 = esp->status_profile.indic_b_19;

  status_msg_.start_index = esp->measurement_contour_descritption.start_index;
  status_msg_.stop_index = esp->measurement_contour_descritption.stop_index;
  status_msg_.index_interval = esp->measurement_contour_descritption.index_interval;
  status_msg_.reserved = esp->measurement_contour_descritption.reseved;
}


void RSL400Interface::verifyConfiguration(DatagramExtendedStatusProfile_rsl400 d_esp)
{
  // Divide by 10 (resolution 0.1°)
  float min_angle_from_esp =
    (static_cast<float>(d_esp.measurement_contour_descritption.start_index) / 10);

  // +1 here to account for the fact that internal calculations are for example from -135° to +135°
  // but actual represntation is from 0° to 269.9° (difference of 0.1°)
  // Divide by 10 (resolution 0.1°)
  float max_angle_from_esp =
    (static_cast<float>(d_esp.measurement_contour_descritption.stop_index + 1) / 10);
  float avg_angle = (min_angle_from_esp + max_angle_from_esp) / 2;

  // Adjust for example from -135° to +135° to 0° to 270°
  min_angle_from_esp = angles::from_degrees(min_angle_from_esp - avg_angle);
  max_angle_from_esp = angles::from_degrees(max_angle_from_esp - avg_angle);

  if (!compareTwoFloats(min_angle_from_esp, laser_scan_.angle_min)) {
    RCLCPP_WARN(
      get_logger(),
      "[Laser Scanner] Current internal minimum angle of %f does not match"
      " the value received from the laser %f. Adjusting internally",
      laser_scan_.angle_min, min_angle_from_esp);
    laser_scan_.angle_min = min_angle_from_esp;
  }
  if (!compareTwoFloats(max_angle_from_esp, laser_scan_.angle_max)) {
    RCLCPP_WARN(
      get_logger(),
      "[Laser Scanner] Current internal maximum angle of %f does not match the value"
      " received from the laser %f. Adjusting internally", laser_scan_.angle_max,
      max_angle_from_esp);
    laser_scan_.angle_max = max_angle_from_esp;
  }

  if (scan_size_ != d_esp.getBeamCount()) {
    RCLCPP_WARN(
      get_logger(),
      "[Laser Scanner] Current internal beam count of %d does not match the value received"
      "from the laser %d. Adjusting internally", scan_size_, d_esp.getBeamCount());
    scan_size_ = d_esp.getBeamCount();
  }

  if (measure_counter_ != 0) {
    RCLCPP_WARN(get_logger(), "[Laser Scanner] Received ExtendedProfile at unexpected timing.");
    measure_counter_ = 0;
  }

  scan_data_.clear();
  scan_data_.resize(scan_size_);
  scan_number_ = d_esp.frame.scan_number;
  configuration_received_ = true;

  // Length 48 is fixed
  if (d_esp.frame.h1.total_length != 48) {
    RCLCPP_WARN(
      get_logger(),
      "[Laser Scanner] Parsing Extended Status Profile of incorrect length %d, expected 48",
      d_esp.frame.h1.total_length);
  }
}
