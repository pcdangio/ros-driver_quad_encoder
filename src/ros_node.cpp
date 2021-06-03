#include "ros_node.h"

#include <sensor_msgs_ext/axis_state.h>

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

    // Set up the publishers.
    ros_node::m_publisher_position = ros_node::m_node.advertise<sensor_msgs_ext::axis_state>("axis_state", 1);

    // Set up the services.
    ros_node::m_service_set_home = private_node.advertiseService("set_home", &ros_node::service_set_home, this);

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

    // Set up tracking for velocity/accleration calculations.
    ros_node::m_prior_time = ros::Time::now();
    ros_node::m_prior_position = 0;
    ros_node::m_prior_velocity = 0;

    // Monitor changes in missed pulses for logging.
    uint64_t n_missed_pulses = 0;

    // Loop.
    while(ros::ok())
    {
        // Create output axis state message.
        sensor_msgs_ext::axis_state message;
        
        // Calculate delta time since last measurement.
        ros::Time current_time = ros::Time::now();
        double_t dt = (current_time - ros_node::m_prior_time).toSec();

        // Get the position measurement from the driver.
        message.position = ros_node::m_driver->get_position();

        // Calculate velocity and acceleration.
        message.velocity = (message.position - ros_node::m_prior_position) / dt;
        message.acceleration = (message.velocity - ros_node::m_prior_velocity) / dt;

        // Update priors.
        ros_node::m_prior_time = current_time;
        ros_node::m_prior_position = message.position;
        ros_node::m_prior_velocity = message.velocity;

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
    // Set the driver's home position.
    ros_node::m_driver->set_home();

    // Reset the prior position to zero to avoid velocity jump.
    ros_node::m_prior_position = 0;

    return true;
}