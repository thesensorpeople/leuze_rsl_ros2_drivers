#include "leuze_phidget_driver.hpp"
#include "rclcpp/rate.hpp"
#include <string>

LeuzePhidgetDriver::LeuzePhidgetDriver():
  Node("leuze_phidget_driver")
{
  pub_show_inputs_ = this->create_publisher<PhidgetInput>("ik_show_inputs",50);
  sub_get_outputs_ = this->create_subscription<leuze_msgs::msg::PhidgetIKOutputMsg>("ik_set_outputs", 10, std::bind(&LeuzePhidgetDriver::getOutputStateCallback, this, std::placeholders::_1));  //static_cast<void(LeuzePhidgetDriver::*)(const leuze_msgs::msg::PhidgetIKOutputMsg::ConstSharedPtr&)>()

  input_state_.resize(_NUMBER_OF_IK_INPUTS_);
  input_subscribers_.resize(_NUMBER_OF_IK_INPUTS_);
  output_state_.resize(_NUMBER_OF_IK_OUTPUTS_);
  output_publishers_.resize(_NUMBER_OF_IK_OUTPUTS_);

  spawnInputSubscribers();
  spawnOutputPublishers();

  RCLCPP_INFO_STREAM(get_logger(), "Waiting for inputs...");

  rclcpp::GenericRate loop_rate(10);
  while(rclcpp::ok())
  {
    publishInputState();
    rclcpp::spin_some(this->shared_from_this()); //Spin until there is nothing for the exexutor to do. ToDo: Test if this works (This is SpinOnce in the ROS1 driver)
    loop_rate.sleep();
  }
}

LeuzePhidgetDriver::~LeuzePhidgetDriver()
{
  //No actions needed here
}

void LeuzePhidgetDriver::readInputStateCallback(const std_msgs::msg::Bool & msg)
{
  //ToDo: Implement this function depending on how the Phidget board provides data to ROS2
  //(it is not clear if we need multiple subscribers for Bool or we we need one subscriber for a more complex data type like in getOutputStateCallback)
#if 0
  input_state_[i] = msg->data?1:0;
  std::cout <<   "Received for input pin : " << i << " state : " << (msg->data?"True":"False") << ". Current input state : ";
  for(auto x : input_state_)
  {
    std::cout << " " << x;
  }
  std::cout << std::endl;
#else
  (void)msg; //Avoid the "unused parameter" warning
#endif
}

void LeuzePhidgetDriver::spawnInputSubscribers()
{
  for (int i=0; i<_NUMBER_OF_IK_INPUTS_; i++)
  {
    std::string topic_name = "/digital_input"+ ( i<=9 ? "0"+ std::to_string(i) : std::to_string(i) );
    auto fn = std::bind( &LeuzePhidgetDriver::readInputStateCallback, this, std::placeholders::_1);
    input_subscribers_[i] = this->create_subscription<std_msgs::msg::Bool>(topic_name, 10, fn);  //static_cast<void(LeuzePhidgetDriver::*)(const std_msgs::msg::Bool::ConstSharedPtr&, int)>()
    RCLCPP_INFO_STREAM(get_logger(), "Spawned input topic subscriber " << i << " for : " << topic_name);
    rclcpp::spin_some(this->shared_from_this()); //Spin until there is nothing for the exexutor to do. ToDo: Test if this works (This is SpinOnce in the ROS1 driver)
  }
}

void LeuzePhidgetDriver::spawnOutputPublishers()
{
  for (int i=0; i<_NUMBER_OF_IK_OUTPUTS_; i++)
  {
    std::string topic_name = "/digital_output"+ ( i<=9 ? "0"+ std::to_string(i) : std::to_string(i)  );
    output_publishers_[i] = this->create_publisher<std_msgs::msg::Bool>(topic_name, 50);
    RCLCPP_INFO_STREAM(get_logger(), "Spawned output topic publisher " << i << " for : " << topic_name);
    rclcpp::spin_some(this->shared_from_this()); //Spin until there is nothing for the exexutor to do. ToDo: Test if this works (This is SpinOnce in the ROS1 driver)
  }
}

void LeuzePhidgetDriver::publishInputState()
{
  leuze_msgs::msg::PhidgetIKInputMsg msg;
  msg.i_0_ossd1= input_state_[0];
  msg.i_1_ossd2= input_state_[1];
  msg.i_2_wf1vio= input_state_[2];
  msg.i_3_wf2vio= input_state_[3];
  msg.i_4_pfvio= input_state_[4];
  msg.header.stamp = this->get_clock()->now();
  pub_show_inputs_->publish(msg);
}

void LeuzePhidgetDriver::publishOutputState()
{
  std_msgs::msg::Bool msg;
  for (int i=0; i<_NUMBER_OF_IK_OUTPUTS_; i++)
  {
    msg.data = output_state_[i];
    output_publishers_[i]->publish(msg);
  }
}

void LeuzePhidgetDriver::getOutputStateCallback(const leuze_msgs::msg::PhidgetIKOutputMsg &msg)
{
  output_state_[0] = msg.o_0_res1;
  output_state_[1] = msg.o_1_f1;
  output_state_[2] = msg.o_2_f2;
  output_state_[3] = msg.o_3_f3;
  output_state_[4] = msg.o_4_f4;
  output_state_[5] = msg.o_5_f5;
  output_state_[6] = msg.o_6_f6;
  publishOutputState();
  rclcpp::spin_some(this->shared_from_this()); //Spin until there is nothing for the exexutor to do. ToDo: Test if this works (This is SpinOnce in the ROS1 driver)
}


