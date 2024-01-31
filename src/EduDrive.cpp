#include "EduDrive.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <fcntl.h>
#include <linux/gpio.h>
#include <sys/ioctl.h>


namespace edu
{

    EduDrive::EduDrive() : Node("edu_drive_node")
    {

    }

    EduDrive::~EduDrive()
    {
        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        {
            (*it)->disable();
            delete *it;
        }
        delete _carrier;
    }

    void EduDrive::initDrive(std::vector<ControllerParams> cp, SocketCAN &can, bool verbosity)
    {
        _verbosity = verbosity;
        _enabled = false;

        _subJoy     = this->create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&EduDrive::joyCallback, this, std::placeholders::_1));
        _subVel     = this->create_subscription<geometry_msgs::msg::Twist>("vel/teleop", 10, std::bind(&EduDrive::velocityCallback, this, std::placeholders::_1));
        _srvEnable  = this->create_service<std_srvs::srv::SetBool>("enable", std::bind(&EduDrive::enableCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        // Publisher of motor shields
        _pubEnabled = this->create_publisher<std_msgs::msg::ByteMultiArray>("enabled", 1);
        _pubRPM     = this->create_publisher<std_msgs::msg::Float32MultiArray>("rpm", 1);

        // Publisher of carrier shield
        _pubTemp         = this->create_publisher<std_msgs::msg::Float32>("temperature", 1);
        _pubVoltageMCU   = this->create_publisher<std_msgs::msg::Float32>("voltageMCU", 1);
        _pubCurrentMCU   = this->create_publisher<std_msgs::msg::Float32>("currentMCU", 1);
        _pubVoltageDrive = this->create_publisher<std_msgs::msg::Float32>("voltageDrive", 1);
        _pubCurrentDrive = this->create_publisher<std_msgs::msg::Float32>("currentDrive", 1);
        _pubIMU          = this->create_publisher<sensor_msgs::msg::Imu>("imu", 1);
        _pubOrientation  = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 1);
		
        _carrier = new CarrierBoard(&can, verbosity);
        
        _vMax = 0.f;

        bool isKinematicsValid = true;
        for (unsigned int i = 0; i < cp.size(); ++i)
        {
            std::vector<MotorParams> motorParams = cp[i].motorParams;

            for (unsigned int j = 0; j < motorParams.size(); ++j)
            {
                isKinematicsValid &= (motorParams[j].kinematics.size()==3);
				}
        }
        if(!isKinematicsValid)
        {
            //std::cout << "#EduDrive Kinematic vectors does not fit to drive concept. Vectors of lenght==3 are expected." << std::endl;
            RCLCPP_INFO_STREAM(this->get_logger(), "#EduDrive Kinematic vectors does not fit to drive concept. Vectors of lenght==3 are expected.");

            exit(1);
        }
        
        for (unsigned int i = 0; i < cp.size(); ++i)
        {
            _mc.push_back(new MotorController(&can, cp[i], verbosity));
            
            for(unsigned int j=0; j<_mc[i]->getMotorParams().size(); j++)
            {
            	std::vector<double> kinematics = _mc[i]->getMotorParams()[j].kinematics;
	         	double kx = kinematics[0];
	         	double kw = kinematics[2];
	         	if(fabs(kx)>1e-3)
	         	{
	            	float vMax = fabs(cp[i].rpmMax / 60.f * M_PI / kx);
	            	if(vMax > _vMax) _vMax = vMax;
	            }
	            if(fabs(kw)>1e-3)
	            {
	            	float omegaMax = fabs(cp[i].rpmMax / 60.f * M_PI / kw);
	            	if(omegaMax > _omegaMax) _omegaMax = omegaMax;
		         }
            }
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "Instanciated robot with vMax: " << _vMax << " m/s and omegaMax: " << _omegaMax << " rad/s");
    }

    void EduDrive::run()
    {
        _lastCmd = this->get_clock()->now();

        rclcpp::TimerBase::SharedPtr timerReceiveCAN = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&EduDrive::receiveCAN, this));
        rclcpp::TimerBase::SharedPtr timerCheckLaggyConnection = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&EduDrive::checkLaggyConnection, this));

        rclcpp::spin(shared_from_this());
        rclcpp::shutdown();
    }

    void EduDrive::enable()
    {
        RCLCPP_INFO(this->get_logger(), "Enabling robot");

        float voltageDrive = _carrier->getVoltageDrive();
        
        if(voltageDrive < 3.0)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Unable to enable motor controllers. Low voltage on motor power supply rail");
            return;
        }

        // This is added for the RPi version using GPIO16 to enable all motor controllers
        /*int fd = open("/sys/class/gpio/gpio16/value", O_WRONLY);
        if (fd == -1)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Unable to enable motor controllers. It the GPIO16 pin configured as output?");
            return;
        }
        write(fd, "1", 1);
        close(fd);*/

        gpio_write("/dev/gpiochip0", 16, 1);

        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        {
            if(!(*it)->isInitialized())
                (*it)->reinit();
            (*it)->enable();
        }
    }

    void EduDrive::disable()
    {
        RCLCPP_INFO(this->get_logger(), "Disabling robot");

        // This is added for the RPi version using GPIO16 to enable all motor controllers
        /*int fd = open("/sys/class/gpio/gpio16/value", O_WRONLY);
        if (fd == -1)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Unable to disable motor controllers. It the GPIO16 pin configured as output?");
            // At this point we go on to send at least the disable command via CAN.
        }
        else
        {
            write(fd, "0", 1);
            close(fd);
        }*/

        gpio_write("/dev/gpiochip0", 16, 0);

        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
            (*it)->disable();
    }

    void EduDrive::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
    {
        // Assignment of joystick axes to motor commands
        float fwd = joy->axes[1];                    // Range of values [-1:1]
        float left = joy->axes[0];                   // Range of values [-1:1]
        float turn = joy->axes[2];                   // Range of values [-1:1]
        float throttle = (joy->axes[3] + 1.0) / 2.0; // Range of values [0:1]

        // Enable movement in the direction of the y-axis only when the button 12 is pressed
        if (!joy->buttons[11])
            left = 0;

        static int32_t btn9Prev = joy->buttons[9];
        static int32_t btn10Prev = joy->buttons[10];

        if (joy->buttons[9] && !btn9Prev)
        {
            disable();
        }
        else if (joy->buttons[10] && !btn10Prev)
        {
            enable();
        }

        btn9Prev = joy->buttons[9];
        btn10Prev = joy->buttons[10];

        float vFwd = throttle * fwd * _vMax;
        float vLeft = throttle * left * _vMax;
        float omega = throttle * turn * _omegaMax;

        controlMotors(vFwd, vLeft, omega);
    }

    void EduDrive::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr cmd)
    {
        controlMotors(cmd->linear.x, cmd->linear.y, cmd->angular.z);
    }

    bool EduDrive::enableCallback(const std::shared_ptr<rmw_request_id_t> header, const std::shared_ptr<std_srvs::srv::SetBool_Request> request, const std::shared_ptr<std_srvs::srv::SetBool_Response> response)
    {
       if(request->data==true)
       {
           RCLCPP_INFO(this->get_logger(), "Enabling robot");
           enable();
       }
       else
       {
           RCLCPP_INFO(this->get_logger(), "Disabling robot");
           disable();
       }
       response->success = true;
       return true;
    }

    void EduDrive::controlMotors(float vFwd, float vLeft, float omega)
    {
        _lastCmd = this->get_clock()->now();
            
        for (unsigned int i = 0; i < _mc.size(); ++i)
        {
            std::vector<double> kinematics0 = _mc[i]->getMotorParams()[0].kinematics;
            std::vector<double> kinematics1 = _mc[i]->getMotorParams()[1].kinematics;
            float w[2];
            w[0] = kinematics0[0] * vFwd + kinematics0[1] * vLeft + kinematics0[2] * omega;
            w[1] = kinematics1[0] * vFwd + kinematics1[1] * vLeft + kinematics1[2] * omega;

            // Convert from rad/s to rpm
            w[0] *= 60.f / M_PI;
            w[1] *= 60.f / M_PI;
            _mc[i]->setRPM(w);
            if (_verbosity)
                //std::cout << "#EduDrive Setting RPM for drive" << i << " to " << w[0] << " " << w[1] << std::endl;
                RCLCPP_INFO_STREAM(this->get_logger(), "#EduDrive Setting RPM for drive" << i << " to " << w[0] << " " << w[1]);
        }
    }

    void EduDrive::receiveCAN()
    {
        float voltageDrive = _carrier->getVoltageDrive();
        
        std_msgs::msg::Float32MultiArray msgRPM;
        std_msgs::msg::ByteMultiArray msgEnabled;

        bool controllersInitialized = true;
        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        {        
	        controllersInitialized = controllersInitialized && (*it)->isInitialized();
	     }
        
        for (std::vector<MotorController *>::iterator it = std::begin(_mc); it != std::end(_mc); ++it)
        {
            float response[2] = {0, 0};
            bool enableState = false;
            if(controllersInitialized)
            {
                if(voltageDrive > 3.0)
                {                    
                    if((*it)->checkConnectionStatus(200))
                    {
                        (*it)->getWheelResponse(response);
                        enableState = (*it)->getEnableState();
                    }
                    else
                    {
                        //std::cout << "#EduDrive Error synchronizing with device" << (*it)->getCanId() << std::endl;
                        RCLCPP_INFO_STREAM(this->get_logger(), "#EduDrive Error synchronizing with device" << (*it)->getCanId());   
                    }
                }
                else
                {
                    //std::cout << "#EduDrive Low voltage on drive power supply rail for device " << (*it)->getCanId() << std::endl;
                    RCLCPP_INFO_STREAM(this->get_logger(), "#EduDrive Low voltage on drive power supply rail for device " << (*it)->getCanId());
                    
                    (*it)->deinit();
                    disable();
                }
            }
            msgRPM.data.push_back(response[0]);
            msgRPM.data.push_back(response[1]);
            msgEnabled.data.push_back(enableState);
        }
        
        _enabled = false;
        if(msgEnabled.data.size()>0)
        {
            _enabled = msgEnabled.data[0];
            for(unsigned int i=1; i<msgEnabled.data.size(); i++)
            {
                _enabled &= msgEnabled.data[i];
            }
        }

        _pubRPM->publish(msgRPM);
        _pubEnabled->publish(msgEnabled);

        std_msgs::msg::Float32 msgTemperature;
        msgTemperature.data = _carrier->getTemperature();
        _pubTemp->publish(msgTemperature);

        std_msgs::msg::Float32 msgVoltageMCU;
        msgVoltageMCU.data = _carrier->getVoltageMCU();
        _pubVoltageMCU->publish(msgVoltageMCU);

        std_msgs::msg::Float32 msgCurrentMCU;
        msgCurrentMCU.data = _carrier->getCurrentMCU();
        _pubCurrentMCU->publish(msgCurrentMCU);

        std_msgs::msg::Float32 msgVoltageDrive;
        msgVoltageDrive.data = voltageDrive;
        _pubVoltageDrive->publish(msgVoltageDrive);

        std_msgs::msg::Float32 msgCurrentDrive;
        msgCurrentDrive.data = _carrier->getCurrentDrive();
        _pubCurrentDrive->publish(msgCurrentDrive);

        double q[4];
        _carrier->getOrientation(q);
        geometry_msgs::msg::PoseStamped msgOrientation;
        //Sequence number not supported in std_msgs::msg::header in ros2
        //static unsigned int seq = 0;
        //msgOrientation.header.seq = seq++;
        msgOrientation.header.stamp = this->get_clock()->now();
        msgOrientation.header.frame_id = "base_link";
        msgOrientation.pose.position.x = 0;
        msgOrientation.pose.position.y = 0;
        msgOrientation.pose.position.z = 0;
        msgOrientation.pose.orientation.w = q[0];
        msgOrientation.pose.orientation.x = q[1];
        msgOrientation.pose.orientation.y = q[2];
        msgOrientation.pose.orientation.z = q[3];
        _pubOrientation->publish(msgOrientation);
    }

    void EduDrive::checkLaggyConnection()
    {
        rclcpp::Duration dt = this->get_clock()->now() - _lastCmd;
        bool lag = (dt.seconds() > 0.5);
        if(lag  && _enabled)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Lag detected ... deactivate motor control");
            disable();
        }
    }

    int EduDrive::gpio_write(const char *dev_name, int offset, int value)
    {
        struct gpiohandle_request rq;
        struct gpiohandle_data data;
        int fd, ret;
        std::cout << "Write value " << value << " to GPIO at offset " << offset << " (OUTPUT mode) on chip " << dev_name << std::endl;
        fd = open(dev_name, O_RDONLY);
        if (fd < 0)
        {
            std::cout << "Unabled to open " << dev_name << ": " << strerror(errno) << std::endl;
            return -1;
        }
        rq.lineoffsets[0] = offset;
        rq.flags = GPIOHANDLE_REQUEST_OUTPUT;
        rq.lines = 1;
        ret = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &rq);
        close(fd);
        if (ret == -1)
        {
            std::cout << "Unable to line handle from ioctl: " << strerror(errno) << std::endl;
            return -1;
        }
        data.values[0] = value;
        ret = ioctl(rq.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
        if (ret == -1)
        {
            std::cout << "Unable to set line value using ioctl: " << strerror(errno) << std::endl;
            return -1;
        }

        close(rq.fd);
        
        return 1;
    }

} // namespace
