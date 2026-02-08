#include "differential_drive_controller/wheels/wheel_handle.hpp"


// Type Constructor
WheelHandle::WheelHandle(
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> vel,
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> pos,
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> cmd
) :
    position(pos),
    velocity(vel),
    command(cmd)
    {} // end of "WheelHandle(...)"


void WheelHandle::set_command(double desired)
{
    // Set the desired command to the interface
    command.get().set_value(desired);

} // end of "set_command(double)"


double WheelHandle::get_position()
{
    return position.get().get_value();

} // end of "get_position()"


double WheelHandle::get_velocity()
{
    return velocity.get().get_value();

} // end of "get_velocity()"


void WheelHandle::set_command(double desired, std::vector<WheelHandle>& handles)
{
    // For every handle, set the command
    for(size_t i = 0; i < handles.size(); i++)
    {
        handles.at(i).set_command(desired);
    }

} // end of "set_command(double desired, std::vector<WheelHandle>& handles)"


double WheelHandle::get_position(std::vector<WheelHandle>& handles)
{
    int num_wheels = static_cast<int>(handles.size());

    double total = 0;

    for(size_t i = 0; i < num_wheels; i++)
    {
        total += handles.at(i).get_position();
    }

    return total / num_wheels;

} // end of "get_position(std::vector<WheelHandle>& handles)"


double WheelHandle::get_velocity(std::vector<WheelHandle>& handles)
{
    // Average out the positions across all wheels
    int num_wheels = static_cast<int>(handles.size());

    double total = 0;

    for(size_t i = 0; i < num_wheels; i++)
    {
        total += handles.at(i).get_velocity();
    }

    return total / num_wheels;

} // end of "get_velocity(std::vector<WheelHandle>& handles)"