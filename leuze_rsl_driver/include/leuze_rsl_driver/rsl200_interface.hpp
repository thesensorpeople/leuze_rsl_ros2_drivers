// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#ifndef LEUZE_RSL_DRIVER__RSL200_INTERFACE_HPP_
#define LEUZE_RSL_DRIVER__RSL200_INTERFACE_HPP_

#include <string>
#include "interface.hpp"
#include "leuze_msgs/msg/extended_status_profile_msg_rsl200.hpp"


class RSL200Interface : public RslInterface<leuze_msgs::msg::ExtendedStatusProfileMsgRsl200>
{
public:
  RSL200Interface(std::string address, std::string port, std::string topic);
  ~RSL200Interface();

protected:
  void resetDefault() override;
  void verifyConfiguration(DatagramExtendedStatusProfile_rsl200 d_esp);
  void parseExtendedStatusProfile(std::basic_string<unsigned char> buffer) override;
};

#endif  // LEUZE_RSL_DRIVER__RSL200_INTERFACE_HPP_
