#ifndef WHEEL_HANDLE_HPP
#define WHEEL_HANDLE_HPP


#include <cmath>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "differential_drive_controller/math/translation_2d.hpp"
#include "differential_drive_controller/math/twist_2d.hpp"


struct WheelHandle
{
    // The position velocity
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity;

    // The position interface
    std::reference_wrapper<const hardware_interface::LoanedStateInterface> position;

    // The command interface
    std::reference_wrapper<hardware_interface::LoanedCommandInterface> command;

    /**
     * @brief Construct a new Wheel Handle object
     * 
     * @param vel `std::reference_wrapper<const hardware_interface::LoanedStateInterface>` The velocity interface
     * @param pos `std::reference_wrapper<const hardware_interface::LoanedStateInterface>` The position interface
     * @param cmd `std::reference_wrapper<hardware_interface::LoanedCommandInterface>` The command interface
     */
    WheelHandle(
        std::reference_wrapper<const hardware_interface::LoanedStateInterface> vel,
        std::reference_wrapper<const hardware_interface::LoanedStateInterface> pos,
        std::reference_wrapper<hardware_interface::LoanedCommandInterface> cmd
    );

    /**
     * @brief Set the command to the wheel
     * 
     * @param desired `double` The desired command
     */
    void set_command(double desired);

    /**
     * @brief Get the position of the wheel
     * 
     * @return `double` The position 
     */
    double get_position();

    /**
     * @brief Get the velocity of the wheel
     * 
     * @return `double` The velocity 
     */
    double get_velocity();

    /**
     * @brief Set the command of an array of wheels
     * 
     * @param desired `double` The desired command
     * @param handles `std::vector<WheelHandle>&` The array of handles
     */
    static void set_command(double desired, std::vector<WheelHandle>& handles);

    /**
     * @brief Get the average position of an array of wheels
     * 
     * @param handles `std::vector<WheelHandle>&` The array of handles
     */
    static double get_position(std::vector<WheelHandle>& handles);

    /**
     * @brief Get the average velocity of an array of wheels
     * 
     * @param handles `std::vector<WheelHandle>&` The array of handles
     */
    static double get_velocity(std::vector<WheelHandle>& handles);

}; // struct WheelHandle

#endif // WHEEL_HANDLE_HPP