#ifndef LEUZE_PHIDGET_DRIVER_H
#define LEUZE_PHIDGET_DRIVER_H

#include <vector>

#include "std_msgs/msg/bool.hpp"

#include "rclcpp/rclcpp.hpp"
#include "leuze_msgs/msg/phidget_ik_input_msg.hpp"
#include "leuze_msgs/msg/phidget_ik_output_msg.hpp"

using PhidgetInput = leuze_msgs::msg::PhidgetIKInputMsg;
using PhidgetOutput = leuze_msgs::msg::PhidgetIKOutputMsg;


class LeuzePhidgetDriver: public rclcpp::Node/*, public std::enable_shared_from_this<LeuzePhidgetDriver>*/
{
public:
  LeuzePhidgetDriver();
  ~LeuzePhidgetDriver();

protected:
  void readInputStateCallback(const std_msgs::msg::Bool &msg);
  void getOutputStateCallback(const PhidgetOutput &msg);
  void spawnInputSubscribers();
  void spawnOutputPublishers();
  void publishInputState();
  void publishOutputState();


private:
  rclcpp::Publisher<PhidgetInput>::SharedPtr pub_show_inputs_;
  rclcpp::Subscription<leuze_msgs::msg::PhidgetIKOutputMsg>::SharedPtr sub_get_outputs_;
  std::vector<int> input_state_;
  std::vector<int> output_state_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> input_subscribers_; 
  std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> output_publishers_;
  const int _NUMBER_OF_IK_INPUTS_ = 5;
  const int _NUMBER_OF_IK_OUTPUTS_= 7;
};

#endif // LEUZE_PHIDGET_DRIVER_H
