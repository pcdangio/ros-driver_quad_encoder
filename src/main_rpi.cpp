#include "ros_node.h"
#include "rpi_driver.h"

int main(int argc, char **argv)
{
    // Initialize the node.
    ros::init(argc, argv, "driver_quad_encoder");

    // Create the driver.
    std::shared_ptr<rpi_driver> driver = std::make_shared<rpi_driver>();

    // Create the node.
    ros_node node(driver);

    // Run the node.
    node.spin();
}
