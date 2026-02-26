#include "differential_drive_controller/control/differential_drive_drivetrain.hpp"


DifferentialDriveDrivetrain::DifferentialDriveDrivetrain(double track_width, double wheel_radius) :
    m_odometry(track_width),
    m_wheel_radius(wheel_radius)
{} // end of "DifferentialDrivetrain(double, double)"


void DifferentialDriveDrivetrain::assign_handles(std::vector<WheelHandle>& left_handles, std::vector<WheelHandle>& right_handles)
{
    m_left_wheels = left_handles;
    m_right_wheels = right_handles;

} // end of "assign_handles(std::vector<WheelHandle>& , std::vector<WheelHandle>& )"


void DifferentialDriveDrivetrain::update_odometry()
{
    // Update the odometry
    m_odometry.update(
        get_left_distance(),
        get_right_distance()
    );

} // end of "update_odometry"


DifferentialDriveOdometry* DifferentialDriveDrivetrain::get_odometry()
{
    return &m_odometry;

} // end of "get_odometry"


DifferentialDriveKinematics* DifferentialDriveDrivetrain::get_kinematics()
{
    return m_odometry.get_kinematics();

} // end of "get_kinematics"


double DifferentialDriveDrivetrain::get_left_distance()
{
    // Return the average distance of all wheels

    // Rads * radius = distance
    return m_wheel_radius * WheelHandle::get_position(m_left_wheels);

} // end of "get_left_distance"


double DifferentialDriveDrivetrain::get_right_distance()
{
    // Return the average distance of all wheels

    // Rads * radius = distance
    return m_wheel_radius * WheelHandle::get_position(m_right_wheels);

} // end of "get_right_distance"


WheelPositions DifferentialDriveDrivetrain::get_wheel_distances()
{
    return WheelPositions(
        get_left_distance(),
        get_right_distance()
    );

} // end of "get_wheel_distances"


double DifferentialDriveDrivetrain::get_left_velocities()
{
    // Return the average linear velocity of all wheels
    return m_wheel_radius * WheelHandle::get_velocity(m_left_wheels);

} // end of "get_left_velocities"


double DifferentialDriveDrivetrain::get_right_velocities()
{
    // Return the average linear velocity of all wheels
    return m_wheel_radius * WheelHandle::get_velocity(m_right_wheels);

} // end of "get_right_velocities"


WheelSpeeds DifferentialDriveDrivetrain::get_wheel_speeds()
{
    return WheelSpeeds(
        get_left_velocities(),
        get_right_velocities()
    );

} // end of "get_wheel_speeds"


Twist2d DifferentialDriveDrivetrain::get_chassis_speed()
{
    return m_odometry.get_kinematics()->to_chassis_speeds(get_wheel_speeds());

} // end of "get_chassis_speed"


void DifferentialDriveDrivetrain::set_left_command(double rads_per_sec)
{
    WheelHandle::set_command(rads_per_sec, m_left_wheels);

} // end of "set_left_command(double)"


void DifferentialDriveDrivetrain::set_right_command(double rads_per_sec)
{
    WheelHandle::set_command(rads_per_sec, m_right_wheels);

} // end of "set_right_command(double)"


void DifferentialDriveDrivetrain::stop()
{
    set_wheel_speeds(0, 0);

} // end of "stop()"


void DifferentialDriveDrivetrain::set_wheel_speeds(double left_rads_per_sec, double right_rads_per_sec)
{
    set_left_command(left_rads_per_sec);
    set_right_command(right_rads_per_sec);

} // end of "set_wheel_speeds(double, double)"


void DifferentialDriveDrivetrain::set_wheel_speeds(WheelSpeeds rads_per_sec)
{
    set_wheel_speeds(
        rads_per_sec.left_velocity,
        rads_per_sec.right_velocity
    );

} // end of "set_wheel_speeds(WheelSpeeds)"


void DifferentialDriveDrivetrain::drive_from_chassis(Twist2d desired, bool is_open_loop, double turn_coeff)
{
    WheelSpeeds wheel_speeds_to_set = is_open_loop ?
        m_odometry.get_kinematics()->to_arbitrary_wheel_speeds(desired, turn_coeff) :
        m_odometry.get_kinematics()->to_wheel_speeds(desired).times(1 / m_wheel_radius);
    
    // `to_wheel_speeds` gets meters per second, but the command is rads per second
    // meters per sec / radius = rads per sec

    set_wheel_speeds(wheel_speeds_to_set);

} // end of "drive_from_chassis(Twist2d, bool)"