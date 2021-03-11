#include "ros_node.h"

#include <geometry_msgs_ext/angle.h>

// CONSTRUCTORS
ros_node::ros_node(const std::shared_ptr<driver>& device_driver)
{
    // Store the device driver.
    ros_node::m_driver = device_driver;

    // Read parameters.
    ros::NodeHandle private_node("~");
    int param_gpio_a = private_node.param<int>("gpio_pin_a", 0);
    int param_gpio_b = private_node.param<int>("gpio_pin_b", 0);
    int param_ppr = private_node.param<int>("ppr", 200);
    ros_node::p_publish_rate = private_node.param<double>("publish_rate", 30);
    ros_node::p_delta_mode = private_node.param<bool>("delta_mode", false);

    // Set up the publishers.
    ros_node::m_publisher_position = ros_node::m_node.advertise<geometry_msgs_ext::angle>("position", 1);

    // Set up the services.
    ros_node::m_service_set_home = ros_node::m_node.advertiseService("set_home", &ros_node::service_set_home, this);

    // Initialize the driver.
    try
    {
        ros_node::m_driver->initialize(static_cast<unsigned int>(param_gpio_a), static_cast<unsigned int>(param_gpio_b), static_cast<unsigned int>(param_ppr));
        ROS_INFO_STREAM("encoder initialized successfully on pins " << param_gpio_a << " and " << param_gpio_b << ".");
    }
    catch (std::exception& e)
    {
        ROS_FATAL_STREAM("failed to initialize driver (" << e.what() << ")");
        ros::shutdown();
    }
}

// METHODS
void ros_node::spin()
{
    // Set up loop rate.
    ros::Rate loop_rate(ros_node::p_publish_rate);

    // Monitor changes in missed pulses for logging.
    uint64_t n_missed_pulses = 0;

    // Loop.
    while(ros::ok())
    {
        // Create angle message from current position (and reset it if in delta mode)
        geometry_msgs_ext::angle message;
        message.angle = ros_node::m_driver->get_position(ros_node::p_delta_mode);

        // Publish the message.
        ros_node::m_publisher_position.publish(message);

        // Log any missed pulses.
        if(n_missed_pulses != ros_node::m_driver->pulses_missed())
        {
            ROS_WARN_STREAM("missed " << ros_node::m_driver->pulses_missed() - n_missed_pulses << " pulses");
            // Update the last known pulses missed.
            n_missed_pulses = ros_node::m_driver->pulses_missed();
        }

        // Sleep until next time iteration.
        loop_rate.sleep();
    }
}

bool ros_node::service_set_home(sensor_msgs_ext::set_axis_homeRequest &request, sensor_msgs_ext::set_axis_homeResponse &response)
{
    ros_node::m_driver->set_home();
    return true;
}