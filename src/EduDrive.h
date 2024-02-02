#ifndef __EDU_DRIVE_H
#define __EDU_DRIVE_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "MotorController.h"
#include "CarrierBoard.h"
#include "PowerManagementBoard.h"

namespace edu
{

/**
 * @class EduDrive
 * @brief Drive interface to EduArt's stackable motor controllers
 * @author Stefan May
 * @date 27.04.2022
 */
class EduDrive : public rclcpp::Node
{
public:
    /**
     * @brief Constructor
     *
     */
    EduDrive();

    /**
     * @brief Destroy the Edu Drive object
     *
     */
    ~EduDrive();

    /**
     * @brief Initialize the Drive
     *
     */
    void initDrive(std::vector<ControllerParams> cp, SocketCAN& can, bool verbosity=false);

    /**
     * @brief Blocking ROS handler method. Call this method to enter the ROS message loop.
     *
     */
    void run();

    /**
     * @brief Enable all drives
     * 
     */
    void enable();

    /**
     * @brief Disable all drives
     * 
     */
    void disable();

    /**
     * @brief Method called by ROS as joystick data is available
     * 
     * @param joy joystick data
     */
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);

    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr cmd);

    void receiveCAN();
    
    void checkLaggyConnection();
    
private:

    int gpio_write(const char *dev_name, int offset, int value);

    void controlMotors(float vFwd, float vLeft, float omega);

    bool enableCallback(const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<std_srvs::srv::SetBool_Request> request, const std::shared_ptr<std_srvs::srv::SetBool_Response> response);

    //rclcpp::Node _node; //This class inherits from rclcpp::Node, no need for variable _node
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr           _subJoy;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       _subVel;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr               _srvEnable;
    rclcpp::Publisher<std_msgs::msg::ByteMultiArray>::SharedPtr      _pubEnabled;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr   _pubRPM;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             _pubTemp;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             _pubVoltageMCU;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             _pubCurrentMCU;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             _pubVoltageDrive;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             _pubCurrentDrive;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr              _pubIMU;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    _pubOrientation;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             _pubVoltagePwrMgmt;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr             _pubCurrentPwrMgmt;

    rclcpp::Time _lastCmd;       // Time elapsed since last call
    std::vector<MotorController*>  _mc;
    CarrierBoard* _carrier;
    PowerManagementBoard* _pwr_mgmt;

    double _vMax;
    double _omegaMax;

    bool _enabled;
    bool _verbosity;
};

} // namespace

#endif //__EDU_DRIVE_H
