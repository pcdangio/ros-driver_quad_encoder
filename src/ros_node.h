/// \file ros_node.h
/// \brief Defines the ros_node class.
#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "driver.h"

#include <ros/ros.h>
#include <sensor_msgs_ext/set_axis_home.h>

#include <chrono>

///
/// \brief Implements the driver's ROS node functionality.
///
class ros_node
{
public:
    // CONSTRUCTORS
    ///
    /// \brief ros_node Initializes the ROS node.
    /// \param device_driver A polymorphic pointer to the device's driver.
    /// \param argc Number of main() args.
    /// \param argv The main() args.
    ///
    ros_node(driver* device_driver, int argc, char **argv);
    ~ros_node();

    // METHODS
    ///
    /// \brief spin Runs the node.
    ///
    void spin();

private:
    // VARIABLES
    ///
    /// \brief m_driver The polymorphic instance of the device's driver.
    ///
    driver* m_driver;
    ///
    /// \brief m_node The node's handle.
    ///
    ros::NodeHandle* m_node;
    ///
    /// \brief m_publisher The publisher for AxisState messages.
    ///
    ros::Publisher m_publisher_state;
    ///
    /// \brief m_service_set_home The service for setting the axis's home position.
    ///
    ros::ServiceServer m_service_set_home;
    ///
    /// \brief m_rate The node's spin rate.
    ///
    ros::Rate* m_rate;

    ///
    /// \brief m_prior_timestamp Stores the timestamp of the prior state reading.
    ///
    ros::Time m_prior_timestamp;
    ///
    /// \brief m_prior_position Stores the position of the prior state reading.
    ///
    double m_prior_position;
    ///
    /// \brief m_prior_velocity Stores the velocity of the prior state reading.
    ///
    double m_prior_velocity;

    ///
    /// \brief set_home A service callback for updating the home position of the axis.
    /// \param request The service's request
    /// \param response
    /// \return
    ///
    bool set_home(sensor_msgs_ext::set_axis_homeRequest &request, sensor_msgs_ext::set_axis_homeResponse &response);
};

#endif // ROS_NODE_H
