#include "differential_drive_controller/wheels/wheel_positions.hpp"


// Default Constructor
WheelPositions::WheelPositions() : 
    left_distance(0),
    right_distance(0)
    {} // end of "WheelPositions()"


// Type Constructor
WheelPositions::WheelPositions(double left, double right) :
    left_distance(left),
    right_distance(right)
    {} // end of "WheelPositions(double, double)"


// Copy Constructor
WheelPositions::WheelPositions(const WheelPositions& other) :
    left_distance(other.left_distance),
    right_distance(other.right_distance)
    {} // end of "WheelPositions(const WheelPositions&)"