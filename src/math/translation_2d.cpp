#include "differential_drive_controller/math/translation_2d.hpp"


// Default Constructor
Translation2d::Translation2d() : 
    x(0), 
    y(0)
    {} // end of "Translation2d()"


// Type Constructor
Translation2d::Translation2d(double x_, double y_) :
    x(x_), 
    y(y_)
    {} // end of "Translation2d(double, double)"


// Copy Constructor
Translation2d::Translation2d(const Translation2d& other) :
    x(other.x),
    y(other.y)
    {} // end of Translation2d(const Translation2d&)


geometry_msgs::msg::Vector3 Translation2d::to_vector() const
{
    // Create (x, y, 0)
    geometry_msgs::msg::Vector3 vector;

    vector.x = x;
    vector.y = y;
    vector.z = 0;

    return vector;

} // end of "to_vector"


geometry_msgs::msg::Point Translation2d::to_point() const
{
    // Create (x, y, 0)
    geometry_msgs::msg::Point point;

    point.x = x;
    point.y = y;
    point.z = 0;

    return point;

} // end of "to_point"


Translation2d Translation2d::rotate_by(double rotation) const
{
    /*
    [x y]^T times rotation matrix
    */
    return Translation2d(
        (x * cos(rotation)) - (y * sin(rotation)),
        (x * sin(rotation)) + (y * cos(rotation))
    );

} // end of "rotate_by(double)"


Translation2d Translation2d::plus(const Translation2d& other) const
{
    // Add x and y respectively
    return Translation2d(
        x + other.x,
        y + other.y
    );

} // end of "plus(const Translation2d& other)"