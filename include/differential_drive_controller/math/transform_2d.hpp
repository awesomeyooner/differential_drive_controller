#ifndef TRANSFORM_2D_HPP
#define TRANSFORM_2D_HPP


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

#include "differential_drive_controller/math/translation_2d.hpp"
#include "differential_drive_controller/math/twist_2d.hpp"


/**
 * @brief Creates a 2D Position Vector and Angle pair to represent
 * a coordinate in 2D space
 */
struct Transform2d
{
    // The x component
    double x;

    // The y component
    double y;

    // The angle in radians
    double theta;

    /**
     * @brief Construct a new Transform 2d object with 0'd values
     * 
     */
    Transform2d();

    /**
     * @brief Construct a new Transform 2d object
     * 
     * @param x_ `double` The x component
     * @param y_ `double` The y component
     * @param theta_ `double` The angle in radians
     */
    Transform2d(double x_, double y_, double theta_);

    /**
     * @brief Construct a new Transform 2d object
     * 
     * @param translation `Translation2d` The position vector
     * @param theta_ `double` The angle in radians
     */
    Transform2d(Translation2d translation, double theta_);

    /**
     * @brief Construct a new Transform 2d object
     * 
     * @param other `const Transform2d&` The other transform to copy from
     */
    Transform2d(const Transform2d& other);

    /**
     * @brief Get the Translation2D (position vector) of this object
     * 
     * @return `Translation2d` The position vector 
     */
    Translation2d get_translation() const;

    /**
     * @brief Get the quaternion of the rotation where theta is yaw in RPY
     * 
     * @return `geometry_msgs::msg::Quaternion` The quaternion 
     */
    geometry_msgs::msg::Quaternion get_quaternion() const;

    /**
     * @brief Transform this object by another. Adds the position vectors
     * together then applies the rotation matrix of this object to the sum, and
     * the new theta is the sum of the two thetas
     * 
     * @param other 
     * @return Transform2d 
     */
    Transform2d transform_by(const Transform2d& other) const;

    /**
     * @brief Increments the current transform by the differential transform
     * object, `Twist2d`
     * 
     * @param twist `Twist2d` The differential transform
     * @return `Transform2d` The transformed transform object 
     */
    Transform2d exponential(Twist2d twist);

}; // Transform2d


#endif // TRANSFORM_2D_HPP