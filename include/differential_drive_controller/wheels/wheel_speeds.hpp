#ifndef WHEEL_SPEEDS_HPP
#define WHEEL_SPEEDS_HPP


/**
 * @brief Data Type to store relative wheel velocitys of chassis
 * 
 */
struct WheelSpeeds
{
    // The left wheel velocity
    double left_velocity;

    // The right wheel velocity
    double right_velocity;

    /**
     * @brief Construct a new Wheel Speeds object with 0'd values
     * 
     */
    WheelSpeeds();

    /**
     * @brief Construct a new Wheel Speeds object
     * 
     * @param left `double` The left speed
     * @param right `double` The right speed
     */
    WheelSpeeds(double left, double right);

    /**
     * @brief Construct a new Wheel Speeds object
     * 
     * @param other `const WheelSpeed&` The other object to copy from 
     */
    WheelSpeeds(const WheelSpeeds& other);

    /**
     * @brief Multiplies the current speeds by a scalar
     * 
     * @param scalar `double` The scalar to multiply by
     * @return `WheelSpeeds` The multiplied wheel speeds 
     */
    WheelSpeeds times(double scalar) const;

}; // struct WheelSpeeds


#endif // WHEEL_SPEEDS_HPP