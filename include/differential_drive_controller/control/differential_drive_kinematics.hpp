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
         * @param track_width `double` The distance between the left and right wheels
         */
        DifferentialDriveKinematics(double track_width);

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
         * @param chassis_speed Desired chassis twist
         * @return WheelSpeeds The left and right wheel speeds
         */
        WheelSpeeds to_wheel_speeds(Twist2d chassis_speed);

        /**
         * @brief Returns the wheel speeds to move at the arbitrary chassis speeds
         * 
         * @param chassis_speed `WheelSpeeds` The arbitrary wheel speeds of the robot
         * @param turn_coefficient `double` How much the `dTheta` element in `chassis_speed
         * affects the turn rate
         * @return WheelSpeeds The left and right wheels peeds
         */
        WheelSpeeds to_arbitrary_wheel_speeds(Twist2d chassis_speed, double turn_coefficient = 1);

        /**
         * @brief Returns the twist given how much each wheel has turned
         * 
         * @param left_distance 
         * @param right_distance 
         * @return `Twist2d` The resulting twist 
         */
        Twist2d to_twist(double left_distance, double right_distance);

        /**
         * @brief Returns a twist for a difference in initial and final
         * 
         * @param initial Initial wheel positions
         * @param final Final wheel positions
         * @return `Twist2d` The resulting twist 
         */
        Twist2d to_twist(WheelPositions initial, WheelPositions final);

    private:

        // The distance between the left and right wheels
        double m_track_width;
        
};

#endif