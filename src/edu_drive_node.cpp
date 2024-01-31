#include <iostream>

#include "EduDrive.h"
#include "rclcpp/rclcpp.hpp"
#include <vector>

int main(int argc, char *argv[])
{
   rclcpp::init(argc, argv);
   auto edu_drive_node = std::make_shared<edu::EduDrive>();

   std::vector<edu::ControllerParams> controllerParams;

   // --- System parameters --------
   std::string canInterface;
   int frequencyScale;
   float inputWeight;
   int maxPulseWidth;
   int timeout;
   
   edu_drive_node->declare_parameter("canInterface", std::string("can0"));
   edu_drive_node->declare_parameter("frequencyScale", 32);
   edu_drive_node->declare_parameter("inputWeight", 0.8f);
   edu_drive_node->declare_parameter("maxPulseWidth", 50);
   edu_drive_node->declare_parameter("timeout", 300);

   canInterface = edu_drive_node->get_parameter("canInterface").as_string();
   frequencyScale = edu_drive_node->get_parameter("frequencyScale").as_int();
   inputWeight = edu_drive_node->get_parameter("inputWeight").as_double();
   maxPulseWidth = edu_drive_node->get_parameter("maxPulseWidth").as_int();
   timeout = edu_drive_node->get_parameter("timeout").as_int();

   // Ensure a proper range for the timeout value
   // A lag more than a second should not be tolerated
   if (timeout < 0 && timeout > 1000)
      timeout = 300;

   float kp;
   float ki;
   float kd;
   int antiWindup;
   int invertEnc;
   int responseMode;

   edu_drive_node->declare_parameter("kp", 0.0f);
   edu_drive_node->declare_parameter("ki", 0.f);
   edu_drive_node->declare_parameter("kd", 0.f);
   edu_drive_node->declare_parameter("antiWindup", 1);
   edu_drive_node->declare_parameter("invertEnc", 0);
   edu_drive_node->declare_parameter("responseMode", 0);

   kp = edu_drive_node->get_parameter("kp").as_double();
   ki = edu_drive_node->get_parameter("ki").as_double();
   kd = edu_drive_node->get_parameter("kd").as_double();
   antiWindup   = edu_drive_node->get_parameter("antiWindup").as_int();
   invertEnc    = edu_drive_node->get_parameter("invertEnc").as_int();
   responseMode = edu_drive_node->get_parameter("responseMode").as_int();

   // -----------------------------

   // --- Controller parameters ---
   int controllers = 0;
   edu_drive_node->declare_parameter("controllers", 0);
   controllers = edu_drive_node->get_parameter("controllers").as_int();

   for(int c=0; c<controllers; c++)
   {
      edu::ControllerParams cp;

      cp.frequencyScale = frequencyScale;
      cp.inputWeight    = inputWeight;
      cp.maxPulseWidth  = maxPulseWidth;
      cp.timeout        = timeout;
      cp.kp             = kp;
      cp.ki             = ki;
      cp.kd             = kd;
      cp.antiWindup     = antiWindup;
      
      std::string controllerID = std::string("controller") + std::to_string(c);
      edu_drive_node->declare_parameter(controllerID + std::string(".canID"), 0);
      edu_drive_node->declare_parameter(controllerID + std::string(".gearRatio"), 0.f);
      edu_drive_node->declare_parameter(controllerID + std::string(".encoderRatio"), 0.f);
      edu_drive_node->declare_parameter(controllerID + std::string(".rpmMax"), 0.f);
      edu_drive_node->declare_parameter(controllerID + std::string(".invertEnc"), 0);

      cp.canID        = edu_drive_node->get_parameter(controllerID + std::string(".canID")).as_int();
      cp.gearRatio    = edu_drive_node->get_parameter(controllerID + std::string(".gearRatio")).as_double();
      cp.encoderRatio = edu_drive_node->get_parameter(controllerID + std::string(".encoderRatio")).as_double();
      cp.rpmMax       = edu_drive_node->get_parameter(controllerID + std::string(".rpmMax")).as_double();
      cp.invertEnc    = edu_drive_node->get_parameter(controllerID + std::string(".invertEnc")).as_int();

      cp.responseMode   = (responseMode==0 ? edu::CAN_RESPONSE_RPM : edu::CAN_RESPONSE_POS);

      // --- Motor parameters ---------
      for(int d=0; d<2; d++)
      {
         std::string driveID = controllerID + std::string(".drive") + std::to_string(d);
         edu_drive_node->declare_parameter(driveID + std::string(".channel"), 0);
         edu_drive_node->declare_parameter<std::vector<double>>(driveID + std::string(".kinematics"), std::vector<double>{0.0,0.0,0.0});

         cp.motorParams[d].channel = edu_drive_node->get_parameter(driveID + std::string(".channel")).as_int();
         cp.motorParams[d].kinematics = edu_drive_node->get_parameter(driveID + std::string(".kinematics")).as_double_array();
      }
      // ------------------------------

      controllerParams.push_back(cp);
   }
   // -------------------------

   edu::SocketCAN can(canInterface);
   can.startListener();
   
   //std::cout << "CAN Interface: " << canInterface << std::endl;
   RCLCPP_INFO_STREAM(edu_drive_node->get_logger(), "CAN Interface: " << canInterface);

   bool verbosity = false;

   edu_drive_node->initDrive(controllerParams, can, verbosity);
   edu_drive_node->run();
}