#ifndef ROS_NODE_H
#define ROS_NODE_H

#include "interface.h"

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <chrono>

class ros_node
{
public:
    // CONSTRUCTORS
    ///
    /// \brief ros_node Initializes the ROS node.
    /// \param device_interface A polymorphic pointer to the device's interface.
    /// \param argc Number of main() args.
    /// \param argv The main() args.
    ///
    ros_node(interface* device_interface, int argc, char **argv);
    ~ros_node();

    // METHODS
    ///
    /// \brief spin Runs the node.
    ///
    void spin();

private:
    // VARIABLES
    ///
    /// \brief m_interface The polymorphic instance of the device's interface.
    ///
    interface* m_interface;
    ///
    /// \brief m_node The node's handle.
    ///
    ros::NodeHandle* m_node;
    ///
    /// \brief m_publisher The publisher for AxisState messages.
    ///
    ros::Publisher m_publisher_state;
    ///
    /// \brief m_publisher_delta The publisher for AxisDelta messages.
    ///
    ros::Publisher m_publisher_delta;
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
    /// \brief m_prior_acceleration Stores the acceleration of the prior state reading.
    ///
    double m_prior_acceleration;

    ///
    /// \brief set_home A service callback for updating the home position of the axis.
    /// \param request The service's request
    /// \param response
    /// \return
    ///
    bool set_home(std_srvs::TriggerRequest &request, std_srvs::TriggerResponse &response);
};

#endif // ROS_NODE_H
