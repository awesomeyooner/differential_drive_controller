#include "differential_drive_controller/wheels/wheel_speeds.hpp"


// Default Constructor
WheelSpeeds::WheelSpeeds() : 
    left_velocity(0),
    right_velocity(0)
    {} // end of "WheelSpeeds()"


// Type Constructor
WheelSpeeds::WheelSpeeds(double left, double right) :
    left_velocity(left),
    right_velocity(right)
    {} // end of "WheelSpeeds(double, double)"


// Copy Constructor
WheelSpeeds::WheelSpeeds(const WheelSpeeds& other) :
    left_velocity(other.left_velocity),
    right_velocity(other.right_velocity)
    {} // end of "WheelSpeeds(const WheelSpeeds&)"


WheelSpeeds WheelSpeeds::times(double scalar) const
{
    return WheelSpeeds(
        left_velocity * scalar,
        right_velocity * scalar
    );

} // end of "times(double)"