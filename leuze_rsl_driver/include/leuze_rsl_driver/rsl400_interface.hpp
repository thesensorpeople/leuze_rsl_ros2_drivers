// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#ifndef LEUZE_RSL_DRIVER__RSL400_INTERFACE_HPP_
#define LEUZE_RSL_DRIVER__RSL400_INTERFACE_HPP_

#include <string>
#include "interface.hpp"
#include "leuze_msgs/msg/extended_status_profile_msg_rsl400.hpp"

class RSL400Interface : public RslInterface<leuze_msgs::msg::ExtendedStatusProfileMsgRsl400>
{
public:
  RSL400Interface(std::string address, std::string port, std::string topic);
  ~RSL400Interface();

protected:
  void resetDefault() override;
  void verifyConfiguration(DatagramExtendedStatusProfile_rsl400 d_esp);
  void parseExtendedStatusProfile(std::basic_string<unsigned char> buffer) override;
};

#endif  // LEUZE_RSL_DRIVER__RSL400_INTERFACE_HPP_
