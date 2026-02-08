#include "differential_drive_controller/math/twist_2d.hpp"


// Default Constructor
Twist2d::Twist2d() : 
    dx(0), 
    dy(0), 
    dTheta(0)
    {} // end of "Twist2d()"


// Type Constructor
Twist2d::Twist2d(double dx_, double dy_, double dTheta_) :
    dx(dx_),
    dy(dy_),
    dTheta(dTheta)
    {} // end of "Twist2d(double, double, double)"


// Type Constructor
Twist2d::Twist2d(geometry_msgs::msg::Twist message) :
    dx(message.linear.x),
    dy(message.linear.y),
    dTheta(message.angular.z)
    {} // end of "Twist2d(geometry_msgs::msg::Twist)"


// Type Constructor
Twist2d::Twist2d(geometry_msgs::msg::TwistStamped message) :
    dx(message.twist.linear.x),
    dy(message.twist.linear.y),
    dTheta(message.twist.angular.z)
    {} // end of "Twist2d(geometry_msgs::msg::TwistStamped)"


// Copy Constructor
Twist2d::Twist2d(const Twist2d& other) :
    dx(other.dx),
    dy(other.dy),
    dTheta(other.dTheta)
    {} // end of "Twist2d(const Twist2d& other)"


geometry_msgs::msg::Twist Twist2d::to_message()
{
    // Create new twist object and populate its members
    geometry_msgs::msg::Twist twist;
    twist.linear.x = dx;
    twist.linear.y = dy;
    twist.angular.z = dTheta;

    return twist;

} // end of "to_message"