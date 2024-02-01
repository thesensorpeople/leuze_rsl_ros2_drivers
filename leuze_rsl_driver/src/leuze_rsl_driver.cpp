// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#include "rclcpp/rclcpp.hpp"

#include "leuze_rsl_driver/rsl400_interface.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 3) {
    std::cerr << "Not enough arguments!" << std::endl;
  }
  std::string address = argv[1];
  std::string port = argv[2];
  std::string topic = argv[3];

  std::cout << "address: " << address << std::endl;
  std::cout << "port: " << port << std::endl;
  std::cout << "topic: " << topic << std::endl;

  auto node = std::make_shared<RSL400Interface>(address, port, topic);

  node->connect();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
