#ifndef WHEEL_POSITIONS_HPP
#define WHEEL_POSITIONS_HPP


/**
 * @brief Data Type to store relative wheel positions of chassis
 * 
 */
struct WheelPositions
{
    // The left wheel position
    double left_distance;

    // The right wheel position
    double right_distance;

    /**
     * @brief Construct a new WheelPositions object with 0'd values
     * 
     */
    WheelPositions();

    /**
     * @brief Construct a new WheelPositions object
     * 
     * @param left `double` The left position
     * @param right `double` The right position
     */
    WheelPositions(double left, double right);

    /**
     * @brief Construct a new WheelPosition object
     * 
     * @param other `const WheelPositions&` The other object to copy from 
     */
    WheelPositions(const WheelPositions& other);
    

}; // struct WheelPositions


#endif // WHEEL_POSITIONS_HPP