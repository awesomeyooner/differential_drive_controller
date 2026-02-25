#include "differential_drive_controller/control/differential_drive_odometry.hpp"


DifferentialDriveOdometry::DifferentialDriveOdometry(double track_width) :
    m_kinematics(track_width)
{} // end of "DifferentialDriveOdometry(double)"


DifferentialDriveOdometry::DifferentialDriveOdometry(double track_width, Transform2d initial_pose) :
    m_kinematics(track_width),
    m_pose(initial_pose)
{} // end of "DifferentialDriveOdometry(double, Transform2d)"


DifferentialDriveOdometry::DifferentialDriveOdometry(DifferentialDriveKinematics kinematics) :
    m_kinematics(kinematics)
{} // end of "DifferentialDriveOdometry(DifferentialDriveKinematics)"


DifferentialDriveOdometry::DifferentialDriveOdometry(DifferentialDriveKinematics kinematics, Transform2d initial_pose) :
    m_kinematics(kinematics),
    m_pose(initial_pose)
{} // end of "DifferentialDriveOdometry(DifferentialDriveKinematics, Transform2d)"


DifferentialDriveKinematics* DifferentialDriveOdometry::get_kinematics()
{
    return &m_kinematics;

} // end of "get_kinematics"


Transform2d DifferentialDriveOdometry::get_pose()
{
    return m_pose;

} // end of "get_pose"


void DifferentialDriveOdometry::set_pose(Transform2d new_pose)
{
    m_pose = new_pose;
     
} // end of "set_pose(Transform2d)"


void DifferentialDriveOdometry::update(WheelPositions current_wheel_positions)
{
    // Calculate the twist between last loop and now
    Twist2d twist = m_kinematics.to_twist(m_prev_wheel_positions, current_wheel_positions);

    Transform2d new_pose = m_pose.exponential(twist);

    // TODO: Overload assignment operator for Transform2d 
    m_pose.x = new_pose.x;
    m_pose.y = new_pose.y;
    m_pose.theta = new_pose.theta;

    // TODO: Overload assignment operator for WheelSpeeds
    m_prev_wheel_positions.left_distance = current_wheel_positions.left_distance;
    m_prev_wheel_positions.right_distance = current_wheel_positions.right_distance;

} // end of "update(WheelPositions)"


void DifferentialDriveOdometry::update(double current_left, double current_right)
{
    // Create a new WheelPositions object and pass it into the normal update function
    WheelPositions current_wheel_positions(current_left, current_right);

    update(current_wheel_positions);

} // end of "update(double, double)"
