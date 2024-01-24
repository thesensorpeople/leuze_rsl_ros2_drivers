#include "leuze_phidget_driver.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  //ToDo: ROS1 uses the init option rclcpp::init_options::AnonymousName). This is currently not supported in the ROS2 driver

    sleep(3); //Wait for phidgets_ik_node to come up and spawn its topics
    
    auto node = std::make_shared<LeuzePhidgetDriver>();

    rclcpp::spin(node); //Pause the program here and keep the node alive until we press CTRL+C

    rclcpp::shutdown();  //Stop ROS2 communications
    return 0;
}
