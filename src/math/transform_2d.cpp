#include "differential_drive_controller/math/transform_2d.hpp"


// Default Constructor
Transform2d::Transform2d() : 
    x(0),
    y(0),
    theta(0)
    {} // end of "Transform2d()"


// Type Constructor
Transform2d::Transform2d(double x_, double y_, double theta_) :
    x(x_),
    y(y_),
    theta(theta_)
    {} // end of "Transform2d(double, double, double)"


// Type Constructor
Transform2d::Transform2d(Translation2d translation, double theta_) :
    x(translation.x),
    y(translation.y),
    theta(theta_)
    {} // end of "Transform2d(Translation2d, double)"


// Copy Constructor
Transform2d::Transform2d(const Transform2d& other) :
    x(other.x),
    y(other.y),
    theta(other.theta)
    {} // end of "Transform2d(const Transform2d& other)"


Translation2d Transform2d::get_translation() const
{
    return Translation2d(x, y);

} // end of "get_translation"


geometry_msgs::msg::Quaternion Transform2d::get_quaternion() const
{
    // The only angle is YAW

    // Use the TF2 package's quaternion for built in conversions
    tf2::Quaternion quaternion;

    quaternion.setRPY(0, 0, theta);

    geometry_msgs::msg::Quaternion native;

    native.w = quaternion.getW();
    native.x = quaternion.getX();
    native.y = quaternion.getY();
    native.z = quaternion.getZ();

    return native;

} // end of "get_quaternion()"


Transform2d Transform2d::transform_by(const Transform2d& other) const
{
    return Transform2d(
        get_translation().plus(other.get_translation().rotate_by(theta)),
        theta + other.theta
    );

} // end of "transform_by(const Transform& other)"


Transform2d Transform2d::exponential(Twist2d twist)
{
    // Shorthands to make math cleaner
    double dx = twist.dx;
    double dy = twist.dy;
    double dTheta = twist.dTheta;

    double sin_theta = sin(dTheta);
    double cos_theta = cos(dTheta);

    // Store the cos / sin expressions since we're using two methods
    double sin_expression;
    double cos_expression;

    // If dTheta is really small, use the taylor series expansion
    // because we're dividing by dTheta
    if(abs(dTheta) < 1E-9)
    {
        sin_expression = 1.0 - (pow(dTheta, 2) / 6.0);
        cos_expression = dTheta / 2;
    }
    // If it's big enough then use the actual equation
    else
    {
        sin_expression = sin_theta / dTheta;
        cos_expression = (1 - cos_theta) / dTheta;
    }

    // Rotation matrix applied to twist
    Transform2d transform(
        (dx * sin_expression) - (dy * cos_expression),
        (dx * cos_expression) + (dy * sin_expression),
        dTheta
    );

    // This transformed by the twist
    return transform_by(transform);

} // end of "exponential(Twist2d)"



