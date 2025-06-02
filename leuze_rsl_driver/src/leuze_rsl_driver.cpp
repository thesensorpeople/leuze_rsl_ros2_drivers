// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#include "rclcpp/rclcpp.hpp"

#if defined(RSL200)
  #include "leuze_rsl_driver/rsl200_interface.hpp"
#elif defined(RSL400)
  #include "leuze_rsl_driver/rsl400_interface.hpp"
#endif

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 4) {
    std::cerr << "Not enough arguments!" << std::endl;
  }
  std::string address = argv[1];
  std::string port = argv[2];
  std::string topic = argv[3];

  std::cout << "address: " << address << std::endl;
  std::cout << "port: " << port << std::endl;
  std::cout << "topic: " << topic << std::endl;

  #if defined(RSL200)
  auto node = std::make_shared<RSL200Interface>(address, port, topic);
  #elif defined(RSL400)
  auto node = std::make_shared<RSL400Interface>(address, port, topic);
  #endif

  node->connect();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
