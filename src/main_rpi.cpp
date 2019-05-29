#include "ros_node.h"
#include "rpi_interface.h"

int main(int argc, char **argv)
{
    // Create the interface.
    rpi_interface* interface = new rpi_interface();

    // Create the node.
    ros_node node(interface, argc, argv);

    // Run the node.
    node.spin();
}
