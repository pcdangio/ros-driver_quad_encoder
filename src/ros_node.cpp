#include "ros_node.h"

#include <sensor_msgs_ext/AxisState.h>

ros_node::ros_node(driver* device_driver, int argc, char **argv)
{
    // Take ownership of the device driver.
    ros_node::m_driver = device_driver;

    // Initialize the node.
    ros::init(argc, argv, "driver_quad_encoder");

    // Get the node's handle.
    ros_node::m_node = new ros::NodeHandle("quad_encoder");

    // Initialize variables.
    ros_node::m_prior_timestamp = ros::Time::now(); // Must be done after ros::init.
    ros_node::m_prior_position = ros_node::m_driver->get_position();
    ros_node::m_prior_velocity = 0;

    // Read parameters.
    ros::NodeHandle private_node("~");
    int param_gpio_a;
    private_node.param<int>("gpio_pin_a", param_gpio_a, 0);
    int param_gpio_b;
    private_node.param<int>("gpio_pin_b", param_gpio_b, 0);
    int param_ppr;
    private_node.param<int>("ppr", param_ppr, 200);
    double param_spin_ratio;
    private_node.param<double>("spin_ratio", param_spin_ratio, 1);
    double param_publish_rate;
    private_node.param<double>("publish_rate", param_publish_rate, 30);

    // Set up the publishers.
    ros_node::m_publisher_state = ros_node::m_node->advertise<sensor_msgs_ext::AxisState>("state", 10);

    // Set up the services.
    ros_node::m_service_set_home = ros_node::m_node->advertiseService("set_home", &ros_node::set_home, this);

    // Set the publishing rate.
    ros_node::m_rate = new ros::Rate(param_publish_rate);

    // Initialize the driver.
    try
    {
        ros_node::m_driver->initialize(static_cast<unsigned int>(param_gpio_a), static_cast<unsigned int>(param_gpio_b), static_cast<unsigned int>(param_ppr), param_spin_ratio);
        ROS_INFO_STREAM("Encoder initialized successfully on pins " << param_gpio_a << " and " << param_gpio_b << ".");
    }
    catch (std::exception& e)
    {
        ROS_FATAL_STREAM(e.what());
        ros::shutdown();
    }
}
ros_node::~ros_node()
{
    delete ros_node::m_rate;
    delete ros_node::m_node;
}

void ros_node::spin()
{
    // Monitor changes in missed pulses for logging.
    long long int n_missed_pulses = 0;

    // Loop.
    while(ros::ok())
    {
        // Read the current position.
        double current_position = ros_node::m_driver->get_position();
        // Read the current timestamp.
        ros::Time current_timestamp = ros::Time::now();
        // Calculate the duration.
        ros::Duration delta_time = current_timestamp - ros_node::m_prior_timestamp;

        // Calculate the delta position.
        double delta_position = current_position - ros_node::m_prior_position;

        // Calculate the current velocity.
        double current_velocity = delta_position / delta_time.toSec();

        // Calculate the delta velocity.
        double delta_velocity = current_velocity - ros_node::m_prior_velocity;

        // Calculate the acceleration.
        double current_acceleration = delta_velocity / delta_time.toSec();

        // Create AxisState message.
        sensor_msgs_ext::AxisState message_state;
        message_state.header.stamp = ros::Time::now();
        message_state.header.frame_id = ros::this_node::getName();
        message_state.position = current_position;
        message_state.velocity = current_velocity;
        message_state.acceleration = current_acceleration;

        // Send the messages.
        ros_node::m_publisher_state.publish(message_state);

        // Update priors.
        ros_node::m_prior_timestamp = current_timestamp;
        ros_node::m_prior_position = current_position;
        ros_node::m_prior_velocity = current_velocity;

        // Log any missed pulses.
        if(n_missed_pulses != ros_node::m_driver->p_pulses_missed())
        {
            ROS_WARN_STREAM("Missed " << ros_node::m_driver->p_pulses_missed() - n_missed_pulses << " pulses.");
            // Update the last known pulses missed.
            n_missed_pulses = ros_node::m_driver->p_pulses_missed();
        }

        // Sleep until next time iteration.
        ros_node::m_rate->sleep();
    }
}

bool ros_node::set_home(sensor_msgs_ext::SetAxisHomeRequest &request, sensor_msgs_ext::SetAxisHomeResponse &response)
{
    ros_node::m_driver->set_home();
    response.success = true;
    return true;
}
