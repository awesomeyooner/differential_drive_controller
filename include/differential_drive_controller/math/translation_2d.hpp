#ifndef TRANSLATION_2D_HPP
#define TRANSLATION_2D_HPP


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


// Creates a 2D Vector representation of a point in space
struct Translation2d
{
    // The x component
    double x;

    // The y component
    double y;

    /**
     * @brief Construct a new Translation 2d object with 0'd values
     * 
     */
    Translation2d();

    /**
     * @brief Construct a new Translation 2d object
     * 
     * @param x_ `double` The x component
     * @param y_ `double` The y component
     */
    Translation2d(double x_, double y_);

    /**
     * @brief Construct a new Translation 2d object
     * 
     * @param other `const Translation2d&` The other object to copy from
     */
    Translation2d(const Translation2d& other);

    /**
     * @brief Converts into the ROS 3D Vector message. `z = 0`
     * 
     * @return `geometry_msgs::msg::Vector3` The converted message 
     */
    geometry_msgs::msg::Vector3 to_vector() const;

    /**
     * @brief Coverts into the ROS 3D Point message. `z = 0`
     * 
     * @return `geometry_msgs::msg::Point` The converted message 
     */
    geometry_msgs::msg::Point to_point() const;
    
    /**
     * @brief Applies the rotation matrix
     * 
     * @param rotation `double` The rotation angle in radians
     * @return `Translation2d` The rotated coordinates 
     */
    Translation2d rotate_by(double rotation) const;

    /**
     * @brief Vector addition with another Translation2d object
     * 
     * @param other `const Translation2d&` The other translation to add
     * @return `Translation2d` The sum of the two translations 
     */
    Translation2d plus(const Translation2d& other) const;


}; // struct Translation2d


#endif // TRANSLATION_2D_HPP