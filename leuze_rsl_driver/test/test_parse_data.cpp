// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#include "rclcpp/rclcpp.hpp"
#include "leuze_rsl_driver/rsl400_interface.hpp"
#include <gtest/gtest.h>


class TestParseData : public testing::Test
{
public:
    std::basic_string<unsigned char> test_buffer;
    //ros::NodeHandle nh;
    static std::shared_ptr<RSL400Interface> node; //Or rclcpp::Node::SharedPtr node

    void SetUp() override
    {
      //nh.setParam("/test_parse_data/scanner_frame","test_frame");
      //interface = new RSL400Interface("192.168.0.1","9990",&nh);

      std::string address = "0.0.0.0";
      std::string port = 0;
      std::string topic = "scan1";

      //node = std::make_shared<RSL400Interface>(address, port, topic);
      node = std::shared_ptr<RSL400Interface>(new RSL400Interface(address, port, topic));
    }

    void TearDown() override
    {

    }

    static std::shared_ptr<RSL400Interface> getNode()
    {
        return node;
    }

};

//Initialize the static member variable:
std::shared_ptr<RSL400Interface> TestParseData::node = nullptr;

TEST_F(TestParseData, test_id_1)
{
    test_buffer.resize(48);
    //ros::NodeHandle nh;
    test_buffer = {0x30 ,0x00 ,0x00 ,0x00 ,0x08 ,0x00 ,0xaf ,0xfe ,  //H1
                   0x04 ,0x15 ,0x02 ,0xc8 , //H2
                   0x01 , 0x00 ,  //ID
                   0x00 ,0x00 , //Block
                   0x6e ,0x29 ,0x05 ,0x00 , //Scan
                   0x01 ,0x01 ,0x02 ,0x80 ,0x00 ,0x00 ,0x10 ,0x80 ,0x6e ,0x29 ,0x05 ,0x00 ,0xe8 ,0x11 ,0x00 ,0xf0 ,0x00 ,0x00 ,0x00 ,0x00 , //Status profile
                   0x00 ,0x00 ,0x8b ,0x0a ,0x01 ,0x00 ,0x00 ,0x00 // Measurement Contour Description
                  }; //Captured from wireshark, the datagram extended status profile
    EXPECT_EQ(node->parseBuffer(test_buffer) , 1);
}

TEST_F(TestParseData, test_id_3)
{
    test_buffer.resize(48);
    //ros::NodeHandle nh;
    test_buffer = {0x30 ,0x00 ,0x00 ,0x00 ,0x08 ,0x00 ,0xaf ,0xfe ,  //H1
                   0x04 ,0x15 ,0x02 ,0xc8 , //H2
                   0x03 , 0x00 ,  //ID
                   0x00 ,0x00 , //Block
                   0x6e ,0x29 ,0x05 ,0x00 , //Scan
                   0x01 ,0x01 ,0x02 ,0x80 ,0x00 ,0x00 ,0x10 ,0x80 ,0x6e ,0x29 ,0x05 ,0x00 ,0xe8 ,0x11 ,0x00 ,0xf0 ,0x00 ,0x00 ,0x00 ,0x00 , //Status profile
                   0x00 ,0x00 ,0x8b ,0x0a ,0x01 ,0x00 ,0x00 ,0x00 // Measurement Contour Description
                  }; //Captured from wireshark, wrong type of datagram for ID 3, but we only test ID control flow here so it's okay
    EXPECT_EQ(node->parseBuffer(test_buffer) , 3);
}

TEST_F(TestParseData, test_id_6)
{
    test_buffer.resize(48);
    //ros::NodeHandle nh;
    test_buffer = {0x30 ,0x00 ,0x00 ,0x00 ,0x08 ,0x00 ,0xaf ,0xfe ,  //H1
                   0x04 ,0x15 ,0x02 ,0xc8 , //H2
                   0x06 , 0x00 ,  //ID
                   0x00 ,0x00 , //Block
                   0x6e ,0x29 ,0x05 ,0x00 , //Scan
                   0x01 ,0x01 ,0x02 ,0x80 ,0x00 ,0x00 ,0x10 ,0x80 ,0x6e ,0x29 ,0x05 ,0x00 ,0xe8 ,0x11 ,0x00 ,0xf0 ,0x00 ,0x00 ,0x00 ,0x00 , //Status profile
                   0x00 ,0x00 ,0x8b ,0x0a ,0x01 ,0x00 ,0x00 ,0x00 // Measurement Contour Description
                  }; //Captured from wireshark, wrong type of datagram for ID 3, but we only test ID control flow here so it's okay
   EXPECT_EQ(node->parseBuffer(test_buffer) , 6);
}

//Unrecogized ID
TEST_F(TestParseData, test_id_4)
{
    test_buffer.resize(48);
    //ros::NodeHandle nh;
    test_buffer = {0x30 ,0x00 ,0x00 ,0x00 ,0x08 ,0x00 ,0xaf ,0xfe ,  //H1
                   0x04 ,0x15 ,0x02 ,0xc8 , //H2
                   0x04 , 0x00 ,  //ID
                   0x00 ,0x00 , //Block
                   0x6e ,0x29 ,0x05 ,0x00 , //Scan
                   0x01 ,0x01 ,0x02 ,0x80 ,0x00 ,0x00 ,0x10 ,0x80 ,0x6e ,0x29 ,0x05 ,0x00 ,0xe8 ,0x11 ,0x00 ,0xf0 ,0x00 ,0x00 ,0x00 ,0x00 , //Status profile
                   0x00 ,0x00 ,0x8b ,0x0a ,0x01 ,0x00 ,0x00 ,0x00 // Measurement Contour Description
                  }; //Captured from wireshark
    EXPECT_EQ(node->parseBuffer(test_buffer) , -1);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);  //Note ROS1 contains a third argument "test_parse_data"
    testing::InitGoogleTest(&argc, argv);

    //ros::AsyncSpinner spinner(1);
    rclcpp::executors::MultiThreadedExecutor spinner;
    spinner.add_node(TestParseData::getNode());
    spinner.spin();
    int ret = RUN_ALL_TESTS();
    spinner.cancel();
    rclcpp::shutdown();
    return ret;
}
