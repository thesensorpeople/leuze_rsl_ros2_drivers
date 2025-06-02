// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#include "leuze_rsl_driver/rsl200_interface.hpp"
#include <angles/angles.h>
#include <algorithm>

RSL200Interface::RSL200Interface(std::string address, std::string port, std::string topic)
: RslInterface(address, port)
{
  // angle range -135° to +135°, step 0.1
  pub_scan_ = this->create_publisher<LaserScan>(topic, 50);
  pub_status_ = this->create_publisher
    <leuze_msgs::msg::ExtendedStatusProfileMsgRsl200>("status_" + topic, 50);
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
    scan_size_ = 1351;  // Default value
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

RSL200Interface::~RSL200Interface()
{
  disconnect();
}


void RSL200Interface::resetDefault()
{
  this->declare_parameter("angle_min", rclcpp::PARAMETER_DOUBLE);  // Default min val (-135°)
  this->declare_parameter("angle_max", rclcpp::PARAMETER_DOUBLE);  // Default min value (+135°)
  this->declare_parameter("scan_time", rclcpp::PARAMETER_DOUBLE);  // 0.025; Default
  this->declare_parameter("range_min", rclcpp::PARAMETER_DOUBLE);  // 0.001; Default
  this->declare_parameter("range_max", rclcpp::PARAMETER_DOUBLE);  // 25.0; Max range 25m

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
    scan_time = 0.025;  // Default value
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
    range_max = 25.0;  // Default value
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


void RSL200Interface::parseExtendedStatusProfile(
  std::basic_string<unsigned char> buffer)
{
  DatagramExtendedStatusProfile_rsl200 * esp =
    reinterpret_cast<DatagramExtendedStatusProfile_rsl200 *>(
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
  status_msg_.image_type = esp->status_profile.imageType;
  status_msg_.op_mode = esp->status_profile.op_mode;
  status_msg_.error_bits = esp->status_profile.errorBits;
  status_msg_.detection_state_bits = esp->status_profile.detectionStateBits;
  status_msg_.triple_sel = esp->status_profile.triple_sel;
  status_msg_.aux = esp->status_profile.aux;
  status_msg_.input_bits = esp->status_profile.inputBits;
  status_msg_.output_bits = esp->status_profile.outputBits;
  status_msg_.volt = esp->status_profile.volt;
  status_msg_.temp = esp->status_profile.temp;
  status_msg_.reserved_0 = 0;
  status_msg_.scan_number = esp->status_profile.scan_number;
  status_msg_.safe_sig = esp->status_profile.safe_sig;
  status_msg_.err_data = esp->status_profile.errDw;

  status_msg_.start_index = esp->measurement_contour_descritption.start_index;
  status_msg_.stop_index = esp->measurement_contour_descritption.stop_index;
  status_msg_.index_interval = esp->measurement_contour_descritption.index_interval;
  status_msg_.reserved_1 = esp->measurement_contour_descritption.reseved;
}


void RSL200Interface::verifyConfiguration(DatagramExtendedStatusProfile_rsl200 d_esp)
{
  // Divide by 5 (resolution 0.2°)
  float min_angle_from_esp =
    (static_cast<float>(d_esp.measurement_contour_descritption.start_index) / 5);

  // For RSL200, the position of 0° is already included in the stop index => no need to add +1 here
  // Divide by 5 (resolution 0.2°)
  float max_angle_from_esp =
    (static_cast<float>(d_esp.measurement_contour_descritption.stop_index) / 5);

  float avg_angle = (min_angle_from_esp + max_angle_from_esp) / 2;

  // Adjust for example from -135° to +135° to 0° to 270°
  min_angle_from_esp = angles::from_degrees(min_angle_from_esp - avg_angle);
  max_angle_from_esp = angles::from_degrees(max_angle_from_esp - avg_angle);

  if (!compareTwoFloats(min_angle_from_esp, laser_scan_.angle_min)) {
    RCLCPP_WARN(
      get_logger(),
      "[Laser Scanner] Current internal minimum angle of %f does not match"
      "the value received from the laser %f. Adjusting internally",
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
      " from the laser %d. Adjusting internally", scan_size_, d_esp.getBeamCount());
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

  // Length 56 is fixed
  if (d_esp.frame.h1.total_length != 56) {
    RCLCPP_WARN(
      get_logger(),
      "[Laser Scanner] Parsing Extended Status Profile of incorrect length %d, expected 56",
      d_esp.frame.h1.total_length);
  }
}
