/// \file ros_node.h
/// \brief Defines the ros_node class.
#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "driver.h"

#include <ros/ros.h>
#include <sensor_msgs_ext/set_axis_home.h>

#include <memory>

/// \brief Implements the driver's ROS node functionality.
class ros_node
{
public:
    // CONSTRUCTORS
    /// \brief Initializes the ROS node.
    /// \param device_driver A polymorphic pointer to the device's driver.
    ros_node(const std::shared_ptr<driver>& device_driver);

    // METHODS
    /// \brief spin Runs the node.
    void spin();

private:
    // DRIVER
    /// \brief The polymorphic instance of the device's driver.
    std::shared_ptr<driver> m_driver;

    // PARAMETERS
    /// \brief The publishing rate in Hz.
    double p_publish_rate;
    /// \brief Indicates if the node should publish angle deltas instead of total angle.
    bool p_delta_mode;

    // ROS
    /// \brief The node's handle.
    ros::NodeHandle m_node;

    // PUBLISHERS
    /// \brief The publisher for encoder position.
    ros::Publisher m_publisher_position;

    // SUBSCRIBERS
    /// \brief The service for setting the axis's home position.
    ros::ServiceServer m_service_set_home;
    /// \brief A service callback for updating the home position of the axis.
    /// \param request The service's request
    /// \param response The service's response
    /// \returns TRUE if the service succeeds, otherwise FALSE.
    bool service_set_home(sensor_msgs_ext::set_axis_homeRequest &request, sensor_msgs_ext::set_axis_homeResponse &response);
};

#endif