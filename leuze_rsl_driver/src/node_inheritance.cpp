// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
  explicit MyNode(std::string name) : Node(name)
  {
    // No action needed here
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MyNode>("test");
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
