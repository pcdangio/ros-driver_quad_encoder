/// \file ros_node.h
/// \brief Defines the ros_node class.
#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "driver.h"

#include <ros/ros.h>
#include <sensor_msgs_ext/set_axis_home.h>

#include <chrono>

/// \brief Implements the driver's ROS node functionality.
class ros_node
{
public:
    // CONSTRUCTORS
    /// \brief Initializes the ROS node.
    /// \param device_driver A polymorphic pointer to the device's driver.
    /// \param argc Number of main() args.
    /// \param argv The main() args.
    ros_node(driver* device_driver, int argc, char **argv);
    ~ros_node();

    // METHODS
    /// \brief spin Runs the node.
    void spin();

private:
    // DRIVER
    /// \brief The polymorphic instance of the device's driver.
    driver* m_driver;

    // ROS
    /// \brief The node's handle.
    ros::NodeHandle* m_node;
    /// \brief The node's spin rate.
    ros::Rate* m_rate;

    // VARIABLES
    /// \brief Stores the timestamp of the prior state reading.
    ros::Time m_prior_timestamp;
    /// \brief Stores the position of the prior state reading.
    double m_prior_position;
    /// \brief Stores the velocity of the prior state reading.
    double m_prior_velocity;

    // PUBLISHERS
    /// \brief The publisher for AxisState messages.
    ros::Publisher m_publisher_state;

    // SUBSCRIBERS
    /// \brief The service for setting the axis's home position.
    ros::ServiceServer m_service_set_home;
    /// \brief A service callback for updating the home position of the axis.
    /// \param request The service's request
    /// \param response
    /// \return
    bool service_set_home(sensor_msgs_ext::set_axis_homeRequest &request, sensor_msgs_ext::set_axis_homeResponse &response);
};

#endif // ROS_NODE_H
