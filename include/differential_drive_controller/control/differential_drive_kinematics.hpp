#ifndef DIFFERENTIAL_DRIVE_KINEMATICS
#define DIFFERENTIAL_DRIVE_KINEMATICS


#include "differential_drive_controller/math/twist_2d.hpp"
#include "differential_drive_controller/math/transform_2d.hpp"
#include "differential_drive_controller/math/translation_2d.hpp"

#include "differential_drive_controller/wheels/wheel_positions.hpp"
#include "differential_drive_controller/wheels/wheel_speeds.hpp"


/**
 * @brief Class handles all forward kinematics of a differential drive
 * 
 */
class DifferentialDriveKinematics
{

    public:

        /**
         * @brief Construct a new Differential Drive Kinematics object
         * 
         * @param track_width_ `double` The distance between the left and right wheels
         */
        DifferentialDriveKinematics(double track_width_);

        /**
         * @brief Returns the chassis twist of the robot given the speeds
         * 
         * @param wheel_speeds `WheelSpeeds` The wheel speeds of the robot
         * @return `Twist2d` The resulting twist 
         */
        Twist2d to_chassis_speeds(WheelSpeeds speeds);

        /**
         * @brief Returns the wheel speeds required to move at the desired chassis speeds
         * 
         * @param chassisSpeed Desired chassis twist
         * @return WheelSpeeds 
         */
        WheelSpeeds to_wheel_speeds(Twist2d chassisSpeed);

        /**
         * @brief Returns the wheel speeds to move at the arbitrary chassis speeds
         * 
         * @param chassisSpeed 
         * @param turnCoefficient 
         * @return WheelSpeeds 
         */
        WheelSpeeds to_arbitrary_wheel_speeds(Twist2d chassisSpeed, double turnCoefficient = 1);

        /**
         * @brief Returns a twist for a difference in initial and final
         * 
         * @param initial Initial wheel positions
         * @param final Final wheel positions
         * @return `Twist2d` The resulting twist 
         */
        Twist2d to_twist(WheelPositions initial, WheelPositions final);

        /**
         * @brief Returns the twist given how much each wheel has turned
         * 
         * @param left_distance 
         * @param right_distance 
         * @return `Twist2d` The resulting twist 
         */
        Twist2d to_twist(double left_distance, double right_distance);

    private:

        // The distance between the left and right wheels
        double track_width;
        
};

#endif