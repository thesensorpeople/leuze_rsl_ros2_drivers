#include "leuze_phidget_driver.hpp"
#include "rclcpp/rate.hpp"
#include <string>

LeuzePhidgetDriver::LeuzePhidgetDriver()
: Node("leuze_phidget_driver")
{
  pub_show_inputs_ = this->create_publisher<PhidgetInput>("ik_show_inputs", 50);
  sub_get_outputs_ = this->create_subscription<leuze_msgs::msg::PhidgetIKOutputMsg>(
    "ik_set_outputs", 10,
    std::bind(&LeuzePhidgetDriver::getOutputStateCallback, this, std::placeholders::_1));

  input_state_.resize(_NUMBER_OF_IK_INPUTS_);
  input_subscribers_.resize(_NUMBER_OF_IK_INPUTS_);
  output_state_.resize(_NUMBER_OF_IK_OUTPUTS_);
  output_publishers_.resize(_NUMBER_OF_IK_OUTPUTS_);
}

LeuzePhidgetDriver::~LeuzePhidgetDriver()
{
  //No actions needed here
}

void LeuzePhidgetDriver::init()
{
  spawnInputSubscribers();
  spawnOutputPublishers();

  RCLCPP_INFO_STREAM(get_logger(), "Waiting for inputs...");

  rclcpp::GenericRate loop_rate(10);
  while (rclcpp::ok()) {
    publishInputState();
    rclcpp::spin_some(this->shared_from_this()); //Spin until there is nothing for the exexutor to do. ToDo: Test if this works (This is SpinOnce in the ROS1 driver)
    loop_rate.sleep();
  }
}

void LeuzePhidgetDriver::readInputStateCallback(const std_msgs::msg::Bool::ConstSharedPtr & msg, const int & i)
{
  input_state_[i] = msg->data ? 1 : 0;
  std::cout << "Received for input pin : " << i << " state : " << (msg->data ? "True" : "False") <<
    ". Current input state : ";
  for (auto x : input_state_) {
    std::cout << " " << x;
  }
  std::cout << std::endl;
}


void LeuzePhidgetDriver::spawnInputSubscribers()
{
  for (int i = 0; i < _NUMBER_OF_IK_INPUTS_; i++) {
    std::string topic_name = "/digital_input" +
      ( i <= 9 ? "0" + std::to_string(i) : std::to_string(i) );

    // ROS2 does not support subscriptions with callbacks having multiple arguments
    // A lambda function is used here to overcome this issue
    // See: https://answers.ros.org/question/308386/ros2-add-arguments-to-callback/

    auto callback_factory = [this](const int & topic_index) {
      return [this, topic_index](const std_msgs::msg::Bool::SharedPtr msg) -> void {
        //printf("topic_name: %s, data: %s\n", a_topic_name.c_str(), msg->data.c_str());
        this->readInputStateCallback(msg, topic_index);
      };
    };

    // Create a subscription.
    // The readInputStateCallback function will be executed whenever data is published to the given topic
    input_subscribers_[i] = create_subscription<std_msgs::msg::Bool>(topic_name, 10, callback_factory(i));

    RCLCPP_INFO_STREAM(
      get_logger(), "Spawned input topic subscriber " << i << " for : " << topic_name);
    rclcpp::spin_some(this->shared_from_this()); //Spin until there is nothing for the exexutor to do. ToDo: Test if this works (This is SpinOnce in the ROS1 driver)
  }
}

void LeuzePhidgetDriver::spawnOutputPublishers()
{
  for (int i = 0; i < _NUMBER_OF_IK_OUTPUTS_; i++) {
    std::string topic_name = "/digital_output" +
      ( i <= 9 ? "0" + std::to_string(i) : std::to_string(i)  );

    // Create a publisher to publish Bool messages to the topic
    // The size of the queue is 50 messages
    output_publishers_[i] = this->create_publisher<std_msgs::msg::Bool>(topic_name, 50);
    RCLCPP_INFO_STREAM(
      get_logger(), "Spawned output topic publisher " << i << " for : " << topic_name);
    rclcpp::spin_some(this->shared_from_this()); //Spin until there is nothing for the exexutor to do. ToDo: Test if this works (This is SpinOnce in the ROS1 driver)
  }
}

void LeuzePhidgetDriver::publishInputState()
{
  leuze_msgs::msg::PhidgetIKInputMsg msg;
  msg.i_0_ossd1 = input_state_[0];
  msg.i_1_ossd2 = input_state_[1];
  msg.i_2_wf1vio = input_state_[2];
  msg.i_3_wf2vio = input_state_[3];
  msg.i_4_pfvio = input_state_[4];
  msg.header.stamp = this->get_clock()->now();
  pub_show_inputs_->publish(msg);
}

void LeuzePhidgetDriver::publishOutputState()
{
  std_msgs::msg::Bool msg;
  for (int i = 0; i < _NUMBER_OF_IK_OUTPUTS_; i++) {
    msg.data = output_state_[i];
    output_publishers_[i]->publish(msg);
  }
}

void LeuzePhidgetDriver::getOutputStateCallback(const leuze_msgs::msg::PhidgetIKOutputMsg & msg)
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
