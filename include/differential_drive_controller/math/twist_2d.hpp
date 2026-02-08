#ifndef TWIST_2D_HPP
#define TWIST_2D_HPP


#include <cmath>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


struct Twist2d
{
    // The differential x component
    double dx;

    // The differential y component
    double dy;

    // The differential angular component
    double dTheta;

    /**
     * @brief Construct a new Twist 2d object with 0'd values
     */
    Twist2d();

    /**
     * @brief Construct a new Twist 2d object
     * 
     * @param dx_ `double` The differential x component
     * @param dy_ `double` The differential y component
     * @param dTheta_ `double` The differential angular component
     */
    Twist2d(double dx_, double dy_, double dTheta_);

    /**
     * @brief Construct a new Twist 2d object from a ROS Twist message
     * 
     * @param message `geometry_msgs::msg::Twist` The ROS message
     */
    Twist2d(geometry_msgs::msg::Twist message);

    /**
     * @brief Construct a new Twist 2d object from a ROS TwistStamped message
     * 
     * @param message `geometry_msgs::msg::TwistStamped` The ROS message
     */
    Twist2d(geometry_msgs::msg::TwistStamped message);

    /**
     * @brief Construct a new Twist 2d object
     * 
     * @param other `const Twist2d&` The other twist to copy from
     */
    Twist2d(const Twist2d& other);

    /**
     * @brief Converts this into the ROS Twist message
     * 
     * @return `geometry_msgs::msg::Twist` The ROS Twist message 
     */
    geometry_msgs::msg::Twist to_message();

}; // struct Pose2d


#endif // TWIST_2D_HPP