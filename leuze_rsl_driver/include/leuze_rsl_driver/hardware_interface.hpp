// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#ifndef LEUZE_RSL_DRIVER__HARDWARE_INTERFACE_HPP_
#define LEUZE_RSL_DRIVER__HARDWARE_INTERFACE_HPP_

#include <string>

#include "leuze_rsl_driver/data_type.hpp"
#include "leuze_rsl_driver/communication.hpp"


// Need a class where the LaserScan msg is populated by ScanData
// Probably same class should parse buffer to header and scan data

template<typename ConnectionType>
class HardwareInterface
{
public:
  HardwareInterface(std::string address, std::string port, DataParser * parser)
  {
    this->connection = new ConnectionType(address, port);
    this->parser = parser;  // probably pass mutex, and conditional var?

    connection->set_handle_read(&DataParser::parseBuffer, parser);
  }

  void connect()
  {
    connection->connect();
    connection->start_read(65507);
  }

  void disconnect()
  {
    // No actions needed
  }

  bool isConnected()
  {
    return true;
  }

  // device specific interface should take care of feeding watchdog, etc
  void get_scan()
  {
    std::cout << "returning scan" << std::endl;
  }

protected:
  Connection * connection;
  DataParser * parser;
};

#endif  // LEUZE_RSL_DRIVER__HARDWARE_INTERFACE_HPP_
