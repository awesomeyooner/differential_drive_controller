#ifndef DIFFERENTIAL_DRIVE_ODOMETRY_HPP
#define DIFFERENTIAL_DRIVE_ODOMETRY_HPP


#include "differential_drive_controller/math/twist_2d.hpp"
#include "differential_drive_controller/math/transform_2d.hpp"
#include "differential_drive_controller/math/translation_2d.hpp"

#include "differential_drive_controller/wheels/wheel_positions.hpp"
#include "differential_drive_controller/wheels/wheel_speeds.hpp"

#include "differential_drive_controller/control/differential_drive_kinematics.hpp"

#include <cmath>
#include <vector>
#include <stdexcept>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


class DifferentialDriveOdometry{

    public:

        /**
         * @brief Construct a new Differential Drive Odometry object with the given track_width
         * 
         * @param track_width `double` The distance between the left and right wheels in meters
         */
        DifferentialDriveOdometry(double track_width);

        /**
         * @brief Construct a new Differential Drive Odometry object with a given track_width and initial position
         * 
         * @param track_width `double` The distance between the left and right wheels in meters
         * @param initial_pose `Transform2d` The initial position of the robot
         */
        DifferentialDriveOdometry(double track_width, Transform2d initial_pose);

        /**
         * @brief Construct a new Differential Drive Odometry object with given kinematics characteristic
         * 
         * @param kinematics `DifferentialDriveKinematics` The kinematics
         */
        DifferentialDriveOdometry(DifferentialDriveKinematics kinematics);

        /**
         * @brief Construct a new Differential Drive Odometry object with given kinematics characteristic
         * and initial position
         * 
         * @param kinematics `DifferentialDriveKinematics` The kinematics
         * @param initial_pose `Transform2d` The initial position of the robot 
         */
        DifferentialDriveOdometry(DifferentialDriveKinematics kinematics, Transform2d initial_pose);

        /**
         * @brief Get the kinematics object as a pointer
         * 
         * @return `DifferentialDriveKinematics*` The pointer to the kinematics object 
         */
        DifferentialDriveKinematics* get_kinematics();

        /**
         * @brief Get the current pose of the robot
         * 
         * @return `Transform2d` The pose of the robot 
         */
        Transform2d get_pose();

        /**
         * @brief Set the new pose of the robot
         * 
         * @param new_pose `Transform2d` The new pose 
         */
        void set_pose(Transform2d new_pose);

        /**
         * @brief Updates the odometry calculations given the new wheel positions
         * 
         * @param current_wheel_positions `WheelPositions` The current wheel positions of the robot
         */
        void update(WheelPositions current_wheel_positions);

        /**
         * @brief Updates the odometry calculations given the new wheel positions
         * 
         * @param current_left `double` The current left distance
         * @param current_right `double` The current right distance
         */
        void update(double current_left, double current_right);

        private:

            // The kinematics model for differential drive
            DifferentialDriveKinematics m_kinematics;

            // The previous loop's wheel positions
            WheelPositions m_prev_wheel_positions;

            // The current pose estimation of the robot
            Transform2d m_pose;

}; // class DifferentialDriveOdometry


#endif