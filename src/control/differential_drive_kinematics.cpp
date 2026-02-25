#include "differential_drive_controller/control/differential_drive_kinematics.hpp"


DifferentialDriveKinematics::DifferentialDriveKinematics(double track_width)
{
    m_track_width = track_width;

} // end of DifferentialDriveKinematics(double)


Twist2d DifferentialDriveKinematics::to_chassis_speeds(WheelSpeeds speeds)
{   
    /*
    Reference:
    https://control.ros.org/humble/doc/ros2_controllers/doc/mobile_robot_kinematics.html
    */

    return Twist2d(
        (speeds.left_velocity + speeds.right_velocity) / 2,
        0,
        (speeds.right_velocity - speeds.left_velocity) / m_track_width
    );

} // end of "to_chassis_speeds(WheelSpeeds)"


WheelSpeeds DifferentialDriveKinematics::to_wheel_speeds(Twist2d chassis_speed)
{
    /*
    Reference:
    https://control.ros.org/humble/doc/ros2_controllers/doc/mobile_robot_kinematics.html
    */

    return WheelSpeeds(
        chassis_speed.dx - (chassis_speed.dTheta * m_track_width / 2),
        chassis_speed.dx + (chassis_speed.dTheta * m_track_width / 2)
    );

} // end of "to_wheel_speeds(Twist2d)"


WheelSpeeds DifferentialDriveKinematics::to_arbitrary_wheel_speeds(Twist2d chassis_speed, double turn_coeff)
{
    // Amount of turn is dependent on `turn_coeff`
    return WheelSpeeds(
        chassis_speed.dx - (chassis_speed.dTheta * turn_coeff),
        chassis_speed.dx + (chassis_speed.dTheta * turn_coeff)
    );

} // end of "to_arbitrary_wheel_speeds(Twist2d, double)"


Twist2d DifferentialDriveKinematics::to_twist(double left_distance, double right_distance)
{
    /*
    Reference:
    https://control.ros.org/humble/doc/ros2_controllers/doc/mobile_robot_kinematics.html
    */

    return Twist2d(
        (left_distance + right_distance) / 2,
        0,
        (right_distance - left_distance) / m_track_width
    );

} // end of "to_twist(double, double)"


Twist2d DifferentialDriveKinematics::to_twist(WheelPositions initial, WheelPositions final)
{
    return to_twist(
        final.left_distance - initial.left_distance, 
        final.right_distance - initial.right_distance
    );   

} // end of "to_twist(WheelPositions, WheelPositions)"


