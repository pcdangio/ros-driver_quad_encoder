#include "ros_node.h"

#include <messages_sensor/AxisState.h>
#include <messages_sensor/AxisDelta.h>

ros_node::ros_node(interface* device_interface, int argc, char **argv)
{
    // Take ownership of the device interface.
    ros_node::m_interface = device_interface;

    // Initialize variables.
    ros_node::m_prior_timestamp = ros::Time::now();
    ros_node::m_prior_position = ros_node::m_interface->get_position();
    ros_node::m_prior_velocity = 0;
    ros_node::m_prior_acceleration = 0;

    // Initialize the node.
    ros::init(argc, argv, "interface_quad_encoder");

    // Get the node's handle.
    ros_node::m_node = new ros::NodeHandle("quad_encoder");

    // Read parameters.
    ros::NodeHandle private_node("~");
    int param_gpio_a;
    private_node.param<int>("gpio_pin_a", param_gpio_a, 0);
    int param_gpio_b;
    private_node.param<int>("gpio_pin_b", param_gpio_b, 0);
    int param_cpr;
    private_node.param<int>("cpr", param_cpr, 200);
    double param_spin_ratio;
    private_node.param<double>("spin_ratio", param_spin_ratio, 1);
    double param_publish_rate;
    private_node.param<double>("publish_rate", param_publish_rate, 30);

    // Set up the publishers.
    ros_node::m_publisher_state = ros_node::m_node->advertise<messages_sensor::AxisState>("state", 10);
    ros_node::m_publisher_delta = ros_node::m_node->advertise<messages_sensor::AxisDelta>("delta", 10);

    // Set up the services.
    ros_node::m_service_set_home = ros_node::m_node->advertiseService("set_home", &ros_node::set_home, this);

    // Set the publishing rate.
    ros_node::m_rate = new ros::Rate(param_publish_rate);

    // Initialize the interface.
    try
    {
        ros_node::m_interface->initialize(static_cast<unsigned int>(param_gpio_a), static_cast<unsigned int>(param_gpio_b), static_cast<unsigned int>(param_cpr), param_spin_ratio);
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
    while(ros::ok())
    {
        // Read the current position.
        double current_position = ros_node::m_interface->get_position();
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

        // Calculate the delta acceleration.
        double delta_acceleration = current_acceleration - ros_node::m_prior_acceleration;

        // Create AxisState message.
        messages_sensor::AxisState message_state;
        message_state.header.stamp = ros::Time::now();
        message_state.header.frame_id = ros::this_node::getName();
        message_state.position = current_position;
        message_state.velocity = current_velocity;
        message_state.acceleration = current_acceleration;

        // Create AxisDelta message.
        messages_sensor::AxisDelta message_delta;
        message_delta.header = message_state.header;
        message_delta.delta_position = delta_position;
        message_delta.delta_velocity = delta_velocity;
        message_delta.delta_acceleration = delta_acceleration;

        // Send the messages.
        ros_node::m_publisher_state.publish(message_state);
        ros_node::m_publisher_delta.publish(message_delta);

        // Update priors.
        ros_node::m_prior_timestamp = current_timestamp;
        ros_node::m_prior_position = current_position;
        ros_node::m_prior_velocity = current_velocity;
        ros_node::m_prior_acceleration = current_acceleration;

        // Sleep until next time iteration.
        ros_node::m_rate->sleep();
    }
}

bool ros_node::set_home(std_srvs::TriggerRequest &request, std_srvs::TriggerResponse &response)
{
    ros_node::m_interface->set_home();
    response.success = true;
    return true;
}
