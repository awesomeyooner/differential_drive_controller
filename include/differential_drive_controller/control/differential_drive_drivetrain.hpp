#ifndef DIFFERENTIAL_DRIVE_DRIVETRAIN_HPP
#define DIFFERENTIAL_DRIVE_DRIVETRAIN_HPP


#include "differential_drive_controller/math/twist_2d.hpp"
#include "differential_drive_controller/math/transform_2d.hpp"
#include "differential_drive_controller/math/translation_2d.hpp"

#include "differential_drive_controller/wheels/wheel_positions.hpp"
#include "differential_drive_controller/wheels/wheel_speeds.hpp"
#include "differential_drive_controller/wheels/wheel_handle.hpp"

#include "differential_drive_controller/control/differential_drive_kinematics.hpp"
#include "differential_drive_controller/control/differential_drive_odometry.hpp"

#include <cmath>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


class DifferentialDriveDrivetrain{

    public:

        /**
         * @brief Creates a new model of a differential drive robot with
         * the specified track width and wheel radius
         * 
         * @param track_width `double` The distance between the left and right wheels in meters 
         * @param wheel_radius `double` The radius of the wheels
         */
        DifferentialDriveDrivetrain(double track_width, double wheel_radius);

        /**
         * @brief Borrows ownership of the command and state interfaces and populates the handles
         * 
         * @param left `std::vector<WheelHandle>&` The left wheel handles
         * @param right `std::vector<WheelHandle>&` The right wheel handles
         */
        void init_handles(std::vector<WheelHandle>& left_handles, std::vector<WheelHandle>& right_handles);

        /**
         * @brief Updates the odometry calculations
         * 
         */
        void update_odometry();

        /**
         * @brief Get the underlying odometry object
         * 
         * @return `DifferentialDriveOdometry*` The odometry object 
         */
        DifferentialDriveOdometry* get_odometry();

        /**
         * @brief Get the underlying kinematics object
         * 
         * @return `DifferentialDriveKinematics*` The kinematics object
         */
        DifferentialDriveKinematics* get_kinematics();

        /**
         * @brief Get the average left distance traveled in meters
         * 
         * @return `double` The distance traveled in meters 
         */
        double get_left_distance();

        /**
         * @brief Get the average right distance traveled in meters
         * 
         * @return `double` The distance traveled in meters 
         */
        double get_right_distance();

        /**
         * @brief Gets the distances the left and right wheels have traveled in meters
         * 
         * @return `WheelPositions` The distances the left and right wheels have traveled in meters 
         */
        WheelPositions get_wheel_distances();

        /**
         * @brief Get the average left velocity in meters per second
         * 
         * @return `double` Velocity in meters per second 
         */
        double get_left_velocities();

        /**
         * @brief Get the average right velocity in meters per second
         * 
         * @return `double` Velocity in meters per second 
         */
        double get_right_velocities();

        /**
         * @brief Gets the velocities of the left and right wheels in meters per second
         * 
         * @return `WheelSpeeds` The left and right wheel velocities in meters per second 
         */
        WheelSpeeds get_wheel_speeds();

        /**
         * @brief Gets the chassis movement vector
         * 
         * @return `Twist2d` The chassis movement vector 
         */
        Twist2d get_chassis_speed();

        /**
         * @brief Sets the speed in rads per second
         * 
         * @param rads_per_sec `double` Desired speed in rads per second
         */
        void set_left_command(double rads_per_sec);

        /**
         * @brief Set the speed in rads per second
         * 
         * @param rads_per_sec `double` Desired speed in rads per second
         */
        void set_right_command(double rads_per_sec);

        /**
         * @brief Set the speed of both sides in rads per second
         * 
         * @param left_rads_per_sec `double` Desired speed in rads per second
         * @param right_rads_per_sec `double` Desired speed in rads per second
         */
        void set_wheel_speeds(double left_rads_per_sec, double right_rads_per_sec);

        /**
         * @brief Set the speed of both sides in rads per second
         * 
         * @param rads_per_sec `double` Desired speed in rads per second
         */
        void set_wheel_speeds(WheelSpeeds rads_per_sec);

        /**
         * @brief Command all motors to 0 rad / s
         * 
         */
        void stop();

        /**
         * @brief Command the robot to move in the given movement vector
         * 
         * @param desired `Twist2d` The commanded movement vector
         * @param is_open_loop `bool = false` Whether or not to use proper units or not
         * @param turn_coeff `double = 1` The turning coefficient for open loop
         */
        void drive_from_chassis(Twist2d desired, bool is_open_loop = false, double turn_coeff = 1);

    private:

        // Odometry object to model differential drive
        DifferentialDriveOdometry m_odometry;

        // Vector of all left wheel handles
        std::vector<WheelHandle> m_left_wheels;

        // Vector of all right wheel handles
        std::vector<WheelHandle> m_right_wheels;

        // The radius of the wheels in meters
        double m_wheel_radius;

}; // class DifferentialDriveDrivetrain


#endif //DIFFERENTIAL_DRIVE_DRIVETRAIN_HPP