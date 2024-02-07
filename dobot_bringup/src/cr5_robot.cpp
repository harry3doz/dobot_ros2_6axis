/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/07
 *
 * <h2><center>&copy; COPYRIGHT 2021 DOBOT CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#include <functional>
#include <memory>
#include <regex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/String.hpp>
#include "cr5_robot.hpp"

DOBOT_BRINGUP_PUBLIC
explicit CR5Robot::CR5Robot(std::shared_ptr<rclcpp::Node> node, std::string name)
    : Node("dobot_action_server"), node_(node), goal_{}, trajectory_duration_(1.0),
{
    action_server_(rclcpp_action::create_server<dobot_msgs::action::MovJ>(
        node_,
        "movj_action_server",
        std::bind(&CR5Robot::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CR5Robot::handle_cancel, this, std::placeholders::_1),
        std::bind(&CR5Robot::handle_accepted, this, std::placeholders::_1)
    ));
    index_ = 0;
    memset(goal_, 0, sizeof(goal_));
    k_servoj_param = std::make_shared<ServoJParam>();
}

CR5Robot::~CR5Robot()
{
    // backend_task_.stop();
    RCLCPP_INFO(node_->get_logger(), "~CR5Robot");
}

void CR5Robot::init()
{
    std::string ip = node_->declare_parameter<std::string>("robot_ip_address", "192.168.5.1");
    trajectory_duration_ = node_->declare_parameter<double>("trajectory_duration", 0.3);
    RCLCPP_INFO(node_->get_logger(), "trajectory_duration : %0.2f", trajectory_duration_);

    commander_ = std::make_shared<CR5Commander>(ip);
    commander_->init();

    // set dobot_arm MOVJ client
    //cli_movj = create_client<dobot_msgs::srv::MovJ>("dobot_bringup/srv/MovJ");

    //service registered here
    // rclcpp::Service<dobot_msgs::srv::EnableRobot>::SharedPtr service = node_->create_service<dobot_msgs::srv::EnableRobot>("EnableRobot", &MG400Robot::enableRobot)
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::EnableRobot>(
        "dobot_bringup/srv/EnableRobot", 
        std::bind(&CR5Robot::enableRobot, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::DisableRobot>(
        "dobot_bringup/srv/DisableRobot",
        std::bind(&CR5Robot::disableRobot, this, std::placeholders::_1, std::placeholders::_2)
        
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ClearError>(
        "dobot_bringup/srv/ClearError",
        std::bind(&CR5Robot::clearError, this, std::placeholders::_1, std::placeholders::_2)
    ));
     server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ResetRobot>(
        "dobot_bringup/srv/ResetRobot",
        std::bind(&CR5Robot::resetRobot, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SpeedFactor>(
        "dobot_bringup/srv/SpeedFactor",
        std::bind(&CR5Robot::speedFactor, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::getErrorID>(
        "dobot_bringup/srv/GetErrorID",
        std::bind(&CR5Robot::getErrorID, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::User>(
        "dobot_bringup/srv/User",
        std::bind(&CR5Robot::user, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::Tool>(
        "dobot_bringup/srv/Tool",
        std::bind(&CR5Robot::tool, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::RobotMode>(
        "dobot_bringup/srv/RobotMode",
        std::bind(&CR5Robot::robotMode, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::PayLoad>(
        "dobot_bringup/srv/PayLoad",
        std::bind(&CR5Robot::payload, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::DO>(
        "dobot_bringup/srv/DO",
        std::bind(&CR5Robot::DO, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::DOExecute>(
        "dobot_bringup/srv/DOExecute",
        std::bind(&CR5Robot::DOExecute, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ToolDO>(
        "dobot_bringup/srv/ToolDO",
        std::bind(&CR5Robot::toolDO, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ToolDOExecute>(
        "dobot_bringup/srv/ToolDOExecute",
        std::bind(&CR5Robot::toolDOExecute, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::AO>(
        "dobot_bringup/srv/AO",
        std::bind(&CR5Robot::AO, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::AOExecute>(
        "dobot_bringup/srv/AOExecute",
        std::bind(&CR5Robot::AOExecute, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::AccJ>(
        "dobot_bringup/srv/AccJ",
        std::bind(&CR5Robot::accJ, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::AccL>(
        "dobot_bringup/srv/AccL",
        std::bind(&CR5Robot::accL, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SpeedJ>(
        "dobot_bringup/srv/SpeedJ",
        std::bind(&CR5Robot::speedJ, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SpeedL>(
        "dobot_bringup/srv/SpeedL",
        std::bind(&CR5Robot::speedL, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::Arch>(
        "dobot_bringup/srv/Arch",
        std::bind(&CR5Robot::arch, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::CP>(
        "dobot_bringup/srv/CP",
        std::bind(&CR5Robot::cp, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::LimZ>(
        "dobot_bringup/srv/LimZ",
        std::bind(&CR5Robot::limZ, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SetArmOrientation>(
        "dobot_bringup/srv/SetArmOrientation",
        std::bind(&CR5Robot::setArmOrientation, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SetPayload>(
        "dobot_bringup/srv/SetPayload",
        std::bind(&CR5Robot::setPayload, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::PositiveSolution>(
        "dobot_bringup/srv/PositiveSolution",
        std::bind(&CR5Robot::positiveSolution, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::InverseSolution>(
        "dobot_bringup/srv/InverseSolution",
        std::bind(&CR5Robot::inverseSolution, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::PowerOn>(
        "dobot_bringup/srv/PowerOn",
        std::bind(&CR5Robot::powerOn, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::RunScript>(
        "dobot_bringup/srv/RunScript",
        std::bind(&CR5Robot::runScript, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::StopScript>(
        "dobot_bringup/srv/StopScript",
        std::bind(&CR5Robot::stopScript, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::PauseScript>(
        "dobot_bringup/srv/PauseScript",
        std::bind(&CR5Robot::pauseScript, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ContinueScript>(
        "dobot_bringup/srv/ContinueScript",
        std::bind(&CR5Robot::continueScript, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::GetHoldRegs>(
        "dobot_bringup/srv/GetHoldRegs",
        std::bind(&CR5Robot::getHoldRegs, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SetHoldRegs>(
        "dobot_bringup/srv/SetHoldRegs",
        std::bind(&CR5Robot::setHoldRegs, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::GetInBits>(
        "dobot_bringup/srv/GetInBits",
        std::bind(&CR5Robot::getInBits, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::GetInRegs>(
        "dobot_bringup/srv/GetInRegs",
        std::bind(&CR5Robot::getInRegs, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::GetCoils>(
        "dobot_bringup/srv/GetCoils",
        std::bind(&CR5Robot::getCoils, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SetCoils>(
        "dobot_bringup/srv/SetCoils",
        std::bind(&CR5Robot::setCoils, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::DI>(
        "dobot_bringup/srv/DI",
        std::bind(&CR5Robot::DI, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ToolDI>(
        "dobot_bringup/srv/ToolDI",
        std::bind(&CR5Robot::toolDI, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::AI>(
        "dobot_bringup/srv/AI",
        std::bind(&CR5Robot::AI, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ToolAI>(
        "dobot_bringup/srv/ToolAI",
        std::bind(&CR5Robot::toolAI, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::DIGroup>(
        "dobot_bringup/srv/DIGroup",
        std::bind(&CR5Robot::DIGroup, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::DOGroup>(
        "dobot_bringup/srv/DOGroup",
        std::bind(&CR5Robot::DOGroup, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::BrakeControl>(
        "dobot_bringup/srv/BrakeControl",
        std::bind(&CR5Robot::brakeControl, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::StartDrag>(
        "dobot_bringup/srv/StartDrag",
        std::bind(&CR5Robot::startDrag, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::StopDrag>(
        "dobot_bringup/srv/StopDrag",
        std::bind(&CR5Robot::stopDrag, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::LoadSwitch>(
        "dobot_bringup/srv/LoadSwitch",
        std::bind(&CR5Robot::loadSwitch, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SetSafeSkin>(
        "dobot_bringup/srv/SetSafeSkin",
        std::bind(&CR5Robot::setSafeSkin, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SetObstacleAvoid>(
        "dobot_bringup/srv/SetObstacleAvoid",
        std::bind(&CR5Robot::setObstacleAvoid, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SetCollisionLevel>(
        "dobot_bringup/srv/SetCollisionLevel",
        std::bind(&CR5Robot::setCollisionLevel, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::EmergencyStop>(
        "dobot_bringup/srv/EmergencyStop",
        std::bind(&CR5Robot::emergencyStop, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::GetTraceStartPose>(
        "dobot_bringup/srv/GetTraceStartPose",
        std::bind(&CR5Robot::getTraceStartPose, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::GetPathStartPose>(
        "dobot_bringup/srv/GetPathStartPose",
        std::bind(&CR5Robot::getPathStartPose, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::HandleTrajPoints>(
        "dobot_bringup/srv/HandleTrajPoints",
        std::bind(&CR5Robot::handleTrajPoints, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::GetSixForceData>(
        "dobot_bringup/srv/GetSixForceData",
        std::bind(&CR5Robot::getSixForceData, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SetCollideDrag>(
        "dobot_bringup/srv/SetCollideDrag",
        std::bind(&CR5Robot::setCollideDrag, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SetTerminalKeys>(
        "dobot_bringup/srv/SetTerminalKeys",
        std::bind(&CR5Robot::setTerminalKeys, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SetTerminal485>(
        "dobot_bringup/srv/SetTerminal485",
        std::bind(&CR5Robot::setTerminal485, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::TCPSpeed>(
        "dobot_bringup/srv/TCPSpeed",
        std::bind(&CR5Robot::TCPSpeed, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::TCPSpeedEnd>(
        "dobot_bringup/srv/TCPSpeedEnd",
        std::bind(&CR5Robot::TCPSpeedEnd, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::GetAngle>(
        "dobot_bringup/srv/GetAndle",
        std::bind(&CR5Robot::getAngle, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::GetPose>(
        "dobot_bringup/srv/GetPose",
        std::bind(&CR5Robot::getPose, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ModbusCreate>(
        "dobot_bringup/srv/ModbusCreate",
        std::bind(&CR5Robot::modbusCreate, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ModbusClose>(
        "dobot_bringup/srv/ModbusClose",
        std::bind(&CR5Robot::modbusClose, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::MovJ>(
        "dobot_bringup/srv/MovJ",
        std::bind(&CR5Robot::movJ, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::MovL>(
        "dobot_bringup/srv/MovL",
        std::bind(&CR5Robot::movL, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::JointMovJ>(
        "dobot_bringup/srv/JointMovJ",
        std::bind(&CR5Robot::jointMovJ, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::Jump>(
        "dobot_bringup/srv/Jump",
        std::bind(&CR5Robot::jump, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::RelMovJ>(
        "dobot_bringup/srv/RelMovJ",
        std::bind(&CR5Robot::relMovJ, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::RelMovL>(
        "dobot_bringup/srv/RelMovL",
        std::bind(&CR5Robot::relMovL, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::RelMovJTool>(
        "dobot_bringup/srv/RelMovJTool",
        std::bind(&CR5Robot::relMovJTool, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::RelMovLTool>(
        "dobot_bringup/srv/RelMovLTool",
        std::bind(&CR5Robot::relMovLTool, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::RelMovJUser>(
        "dobot_bringup/srv/RelMovJUser",
        std::bind(&CR5Robot::relMovJUser, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::RelMovLUser>(
        "dobot_bringup/srv/RelMovLUser",
        std::bind(&CR5Robot::relMovLUser, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::RelJointMovJ>(
        "dobot_bringup/srv/RelJointMovJ",
        std::bind(&CR5Robot::relJointMovJ, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::MovLIO>(
        "dobot_bringup/srv/MovLIO",
        std::bind(&CR5Robot::movLIO, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::MovJIO>(
        "dobot_bringup/srv/MovJIO",
        std::bind(&CR5Robot::movJIO, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::Arc>(
        "dobot_bringup/srv/Arc",
        std::bind(&CR5Robot::arc, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::Circle>(
        "dobot_bringup/srv/Circle",
        std::bind(&CR5Robot::circle, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ServoJ>(
        "dobot_bringup/srv/ServoJ",
        std::bind(&CR5Robot::servoJ, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ServoP>(
        "dobot_bringup/srv/ServoP",
        std::bind(&CR5Robot::servoP, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::Sync>(
        "dobot_bringup/srv/Sync",
        std::bind(&CR5Robot::sync, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::StartTrace>(
        "dobot_bringup/srv/StartTrace",
        std::bind(&CR5Robot::startTrace, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::StartPath>(
        "dobot_bringup/srv/StartPath",
        std::bind(&CR5Robot::startPath, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::StartFCTrace>(
        "dobot_bringup/srv/StartFCTrace",
        std::bind(&CR5Robot::startFCTrace, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::MoveJog>(
        "dobot_bringup/srv/MoveJog",
        std::bind(&CR5Robot::moveJog, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::Wait>(
        "dobot_bringup/srv/Wait",
        std::bind(&CR5Robot::wait, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::Continues>(
        "dobot_bringup/srv/Continues",
        std::bind(&CR5Robot::continues, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::Pause>(
        "dobot_bringup/srv/Pause",
        std::bind(&CR5Robot::pause, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::TCPRealData>(
        "dobot_bringup/srv/TCPRealData",
        std::bind(&CR5Robot::tcpRealData, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::TCPDashboard>(
        "dobot_bringup/srv/TCPDashboard",
        std::bind(&CR5Robot::tcpDashboard, this, std::placeholders::_1, std::placeholders::_2)
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ServoJParam>(
        "dobot_bringup/srv/ServoJParam",
        std::bind(&CR5Robot::servoJParam, this, std::placeholders::_1, std::placeholders::_2)
    ));

    // For now, assuming CR5Robot is a class derived from rclcpp::Node
    // backend_task_timer_ = create_wall_timer(std::chrono::milliseconds(1500), std::bind(&CR5Robot::backendTask, this));

    // action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(this,"follow_joint_trajectory",
    //     std::bind(&CR5Robot::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    //     std::bind(&CR5Robot::handle_cancel, this, std::placeholders::_1),
    //     std::bind(&CR5Robot::handle_accepted, this, std::placeholders::_1)
    // );
    
    // rclcpp::spin(this->get_node_base_interface());
}

// void CR5Robot::feedbackHandle()
// {
//     control_msgs::msg::FollowJointTrajectoryFeedback feedback;

//     double current_joints[6];
//     getJointState(current_joints);

//     for (uint32_t i = 0; i < 6; i++)
//     {
//         feedback.joint_names.push_back(std::string("joint") + std::to_string(i + 1));
//         feedback.actual.positions.push_back(current_joints[i]);
//         feedback.desired.positions.push_back(goal_[i]);
//     }

//     // Implement logic to publish the feedback
//     // handle.publishFeedback(feedback);
// }

// void CR5Robot::feedbackHandle(std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Feedback> handle){}
// rclcpp_action::GoalResponse CR5Robot::handle_goal(const std::array<unsigned char, 16>& uuid, std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> handle){}
// rclcpp_action::CancelResponse CR5Robot::handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> handle){}
// void CR5Robot::handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> handle){}

rclcpp_action::GoalResponse CR5Robot::handle_goal(const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const dobot_msgs::action::MovJ::Goal> goal){
        RCLCPP_INFO(
                        this->get_logger(),
                        "Received goal request with pose: x=%f, y=%f, z=%f, rx=%f, ry=%f, rz=%f",
                        goal->pose.x, goal->pose.y, goal->pose.z,
                        goal->pose.rx, goal->pose.ry, goal->pose.rz);

        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CR5Robot::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dobot_msgs::action::MovJ>> goal_handle){
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
}

void CR5Robot::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dobot_msgs::action::MovJ>> goal_handle){
    // FIXME: 
    // this does not work

    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    using namespace std::placeholders;
    std::thread{std::bind(&CR5Robot::execute, this, _1), goal_handle}.detach();
}

void CR5Robot::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dobot_msgs::action::MovJ>> handle){

    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto target_pose = handle->get_goal()->pose;
    auto feedback = std::make_shared<dobot_msgs::action::MovJ::Feedback>();
 
    dobot_msgs::srv::MovJ::Request movj_req;
    movj_req.x = target_pose.x;
    movj_req.y = target_pose.y;
    movj_req.z = target_pose.z;
    movj_req.a = target_pose.rx;
    movj_req.b = target_pose.ry;
    movj_req.c = target_pose.rz;
    std::shared_ptr<dobot_msgs::srv::MovJ::Response> response;
    bool success = this->movJ(std::make_shared<dobot_msgs::srv::MovJ::Request>(movj_req), response);
    //set_mov_j.call(mov_j_req);
    // commander_->movJ(target_pose.x, target_pose.y, target_pose.z, target_pose.rx, target_pose.ry, target_pose.rz);

    auto result = std::make_shared<dobot_msgs::action::MovJ::Result>();

    double current_val[5];
    this->getToolVectorActual(current_val);

    while(rclcpp::ok()){

        // Check if there is a cancel request
        if (handle->is_canceling()) {
        result->result = false;
        handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
        }

        // Update feedback
        this->getToolVectorActual(current_val);
        feedback->current_pose.x = current_val[0];
        feedback->current_pose.y = current_val[1];
        feedback->current_pose.z = current_val[2];
        feedback->current_pose.rx = current_val[3];
        feedback->current_pose.ry = current_val[4];
        feedback->current_pose.rz = current_val[5];

        // Publish feedback
        handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback");

        // Check if goal is done
        if (rclcpp::ok() &&
            fabs(target_pose.x - current_val[0]) <= 1.0 && fabs(target_pose.y - current_val[1]) <= 1.0 &&
            fabs(target_pose.z - current_val[2]) <= 1.0 && fabs(target_pose.rx - current_val[3]) <= 1.0 &&
            fabs(target_pose.rx - current_val[3]) <= 1.0 && fabs(target_pose.rx - current_val[4]) <= 1.0 &&
            fabs(target_pose.rx - current_val[5]) <= 1.0) {
                result->result = true;
                handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                break;
        }   

        loop_rate.sleep();
    }

}
// void CR5Robot::feedbackHandle(const ros::TimerEvent& tm,
//                               ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
// {
//     control_msgs::FollowJointTrajectoryFeedback feedback;

//     double current_joints[6];
//     getJointState(current_joints);

//     for (uint32_t i = 0; i < 6; i++)
//     {
//         feedback.joint_names.push_back(std::string("joint") + std::to_string(i + 1));
//         feedback.actual.positions.push_back(current_joints[i]);
//         feedback.desired.positions.push_back(goal_[i]);
//     }

//     handle.publishFeedback(feedback);
// }

// void CR5Robot::moveHandle(const ros::TimerEvent& tm,
//                           ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
// {
//     control_msgs::FollowJointTrajectoryGoalConstPtr trajectory = handle.getGoal();

//     if (index_ < trajectory->trajectory.points.size())
//     {
//         auto point = trajectory->trajectory.points[index_].positions;
//         double tmp[6];
//         for (uint32_t i = 0; i < 6; i++)
//         {
//             tmp[i] = point[i] * 180.0 / 3.1415926;
//         }

//         dobot_bringup::ServoJ srv;
//         srv.request.j1 = tmp[0];
//         srv.request.j2 = tmp[1];
//         srv.request.j3 = tmp[2];
//         srv.request.j4 = tmp[3];
//         srv.request.j5 = tmp[4];
//         srv.request.j6 = tmp[5];
//         servoJ(srv.request, srv.response);
//         index_++;
//     }
//     else
//     {
// #define OFFSET_VAL 0.01
//         double current_joints[6];
//         getJointState(current_joints);
//         if ((current_joints[0] >= goal_[0] - OFFSET_VAL) && (current_joints[0] <= goal_[0] + OFFSET_VAL) &&
//             (current_joints[1] >= goal_[1] - OFFSET_VAL) && (current_joints[1] <= goal_[1] + OFFSET_VAL) &&
//             (current_joints[2] >= goal_[2] - OFFSET_VAL) && (current_joints[2] <= goal_[2] + OFFSET_VAL) &&
//             (current_joints[3] >= goal_[3] - OFFSET_VAL) && (current_joints[3] <= goal_[3] + OFFSET_VAL) &&
//             (current_joints[4] >= goal_[4] - OFFSET_VAL) && (current_joints[4] <= goal_[4] + OFFSET_VAL) &&
//             (current_joints[5] >= goal_[5] - OFFSET_VAL) && (current_joints[5] <= goal_[5] + OFFSET_VAL))
//         {
//             timer_.stop();
//             movj_timer_.stop();
//             handle.setSucceeded();
//         }
//     }
// }

// void CR5Robot::goalHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
// {
//     index_ = 0;
//     for (uint32_t i = 0; i < 6; i++)
//     {
//         goal_[i] = handle.getGoal()->trajectory.points[handle.getGoal()->trajectory.points.size() - 1].positions[i];
//     }
//     timer_ = control_nh_.createTimer(ros::Duration(1.0), boost::bind(&CR5Robot::feedbackHandle, this, _1, handle));
//     movj_timer_ = control_nh_.createTimer(ros::Duration(trajectory_duration_),
//                                           boost::bind(&CR5Robot::moveHandle, this, _1, handle));
//     timer_.start();
//     movj_timer_.start();
//     handle.setAccepted();
// }

// void CR5Robot::cancelHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
// {
//     timer_.stop();
//     movj_timer_.stop();
//     handle.setSucceeded();

void CR5Robot::getJointState(double* point)
{
    commander_->getCurrentJointStatus(point);
}

bool CR5Robot::isEnable() const
{
    return commander_->isEnable();
}

bool CR5Robot::isConnected() const
{
    return commander_->isConnected();
}

int CR5Robot::robotStatus() const
{
    return commander_->getRobotMode();
}

void CR5Robot::getToolVectorActual(double* val)
{
    commander_->getToolVectorActual(val);
}

/*
 *----------------------------------------------------------------------------------------------------------------------
 *                                                  dashboard
 *----------------------------------------------------------------------------------------------------------------------
 */

bool CR5Robot::enableRobot(//  std::shared_ptr<rmw_request_id_s> request_header,
    const  std::shared_ptr<dobot_msgs::srv::EnableRobot::Request> request, 
    std::shared_ptr<dobot_msgs::srv::EnableRobot::Response> response)
{
    try
    {
        const char* cmd = "EnableRobot()";
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const std::exception& err)
    {
        response->res = -1;
        return false;
    }
}

bool CR5Robot::disableRobot(
    const std::shared_ptr<dobot_msgs::srv::DisableRobot::Request> request, 
    std::shared_ptr<dobot_msgs::srv::DisableRobot::Response> response)
{
    try
    {
        const char* cmd = "DisableRobot()";
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const std::exception& err)
    {
        response->res = -1;
        return false;
    }
}

bool CR5Robot::clearError(
    const std::shared_ptr<dobot_msgs::srv::ClearError::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ClearError::Response> response)
{
    try
    {
        const char* cmd = "ClearError()";
        commander_->dashboardDoCmd(cmd, response->res);
        response->res = 0;
        return true;
    }
    catch (const std::exception& err)
    {
        response->res = -1;
        return false;
    }
}

bool CR5Robot::resetRobot(
    const std::shared_ptr<dobot_msgs::srv::ResetRobot::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ResetRobot::Response> response)
{
    try
    {
        const char* cmd = "ResetRobot()";
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::speedFactor(
    const std::shared_ptr<dobot_msgs::srv::SpeedFactor::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SpeedFactor::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SpeedFactor(%d)", request->ratio);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::user(
    const std::shared_ptr<dobot_msgs::srv::User::Request> request, 
    std::shared_ptr<dobot_msgs::srv::User::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "User(%d)", request->index);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::tool(
    const std::shared_ptr<dobot_msgs::srv::Tool::Request> request, 
    std::shared_ptr<dobot_msgs::srv::Tool::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Tool(%d)", request->index);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::robotMode(
    const std::shared_ptr<dobot_msgs::srv::RobotMode::Request> request, 
    std::shared_ptr<dobot_msgs::srv::RobotMode::Response> response)
{
    try
    {
        const char* cmd = "RobotMode()";

        std::vector<std::string> result;
        commander_->dashboardDoCmd(cmd, response->res, result);
        if (result.empty())
            throw std::logic_error("robotMode : Empty string");
        response->mode = str2Int(result[0].c_str());
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
    catch (const std::exception& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::payload(
    const std::shared_ptr<dobot_msgs::srv::PayLoad::Request> request, 
    std::shared_ptr<dobot_msgs::srv::PayLoad::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "PayLoad(%0.3f,%0.3f)", request->weight, request->inertia);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::DO(
    const std::shared_ptr<dobot_msgs::srv::DO::Request> request, 
    std::shared_ptr<dobot_msgs::srv::DO::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "DO(%d,%d)", request->index, request->status);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::DOExecute(
    const std::shared_ptr<dobot_msgs::srv::DOExecute::Request> request, 
    std::shared_ptr<dobot_msgs::srv::DOExecute::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "DO(%d,%d)", request->index, request->status);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::toolDO(
    const std::shared_ptr<dobot_msgs::srv::ToolDO::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ToolDO::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "ToolDO(%d,%d)", request->index, request->status);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::toolDOExecute(
    const std::shared_ptr<dobot_msgs::srv::ToolDOExecute::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ToolDOExecute::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "ToolDOExecute(%d,%d)", request->index, request->status);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::AO(
    const std::shared_ptr<dobot_msgs::srv::AO::Request> request, 
    std::shared_ptr<dobot_msgs::srv::AO::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AO(%d,%d)", request->index, request->status);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::AOExecute(
    const std::shared_ptr<dobot_msgs::srv::AOExecute::Request> request, 
    std::shared_ptr<dobot_msgs::srv::AOExecute::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AO(%d,%0.3f)", request->index, static_cast<float>(request->value));
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::accJ(
    const std::shared_ptr<dobot_msgs::srv::AccJ::Request> request, 
    std::shared_ptr<dobot_msgs::srv::AccJ::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AccJ(%d)", request->r);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::accL(
    const std::shared_ptr<dobot_msgs::srv::AccL::Request> request, 
    std::shared_ptr<dobot_msgs::srv::AccL::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "AccL(%d)", request->r);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::speedJ(
    const std::shared_ptr<dobot_msgs::srv::SpeedJ::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SpeedJ::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SpeedJ(%d)", request->r);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::speedL(
    const std::shared_ptr<dobot_msgs::srv::SpeedL::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SpeedL::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SpeedL(%d)", request->r);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::arch(
    const std::shared_ptr<dobot_msgs::srv::Arch::Request> request, 
    std::shared_ptr<dobot_msgs::srv::Arch::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Arch(%d)", request->index);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::cp(
    const std::shared_ptr<dobot_msgs::srv::CP::Request> request, 
    std::shared_ptr<dobot_msgs::srv::CP::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "CP(%d)", request->r);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::limZ(
    const std::shared_ptr<dobot_msgs::srv::LimZ::Request> request, 
    std::shared_ptr<dobot_msgs::srv::LimZ::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "LimZ(%d)", request->value);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::setArmOrientation(
    const std::shared_ptr<dobot_msgs::srv::SetArmOrientation::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SetArmOrientation::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetArmOrientation(%d,%d,%d,%d)", request->l_or_r, request->u_or_d, request->f_or_n, request->config6);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::powerOn(
    const std::shared_ptr<dobot_msgs::srv::PowerOn::Request> request, 
    std::shared_ptr<dobot_msgs::srv::PowerOn::Response> response)
{
    try
    {
        const char* cmd = "PowerOn()";
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::runScript(
    const std::shared_ptr<dobot_msgs::srv::RunScript::Request> request, 
    std::shared_ptr<dobot_msgs::srv::RunScript::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "RunScript(%s)", request->project_name.c_str());
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::stopScript(
    const std::shared_ptr<dobot_msgs::srv::StopScript::Request> request, 
    std::shared_ptr<dobot_msgs::srv::StopScript::Response> response)
{
    try
    {
        const char* cmd = "StopScript()";
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::pauseScript(
    const std::shared_ptr<dobot_msgs::srv::PauseScript::Request> request, 
    std::shared_ptr<dobot_msgs::srv::PauseScript::Response> response)
{
    try
    {
        const char* cmd = "PauseScript()";
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::modbusCreate(
    const std::shared_ptr<dobot_msgs::srv::ModbusCreate::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ModbusCreate::Response> response)
{
    try
    {

        char cmd[300];
        std::vector<std::string> result;
        snprintf(cmd, sizeof(cmd), "ModbusCreate(%s,%d,%d,%d)", request->ip.c_str(), 
                    request->port, request->slave_id,
                    request->is_rtu);
        cmd[sizeof(cmd) - 1] = 0;
        commander_->dashboardDoCmd(cmd, response->res, result);
        if (result.empty())
            throw std::logic_error("Haven't recv any result");
        response->index = str2Int(result[0].c_str());
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        response->index = -1;
        return false;
    }
    catch (const std::exception& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        response->index = -1;
        return false;
    }
}

bool CR5Robot::modbusClose(
    const std::shared_ptr<dobot_msgs::srv::ModbusClose::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ModbusClose::Response> response)
{
    try
    {

        char cmd[300];
        std::vector<std::string> result;
        snprintf(cmd, sizeof(cmd), "ModbusClose(%d)", request->index);
        cmd[sizeof(cmd) - 1] = 0;
        commander_->dashboardDoCmd(cmd, response->res, result);
        if (result.empty())
            throw std::logic_error("Haven't recv any result");
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
    catch (const std::exception& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::setHoldRegs(
    const std::shared_ptr<dobot_msgs::srv::SetHoldRegs::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SetHoldRegs::Response> response)
{
    try
    {
        char cmd[200];
        std::vector<std::string> result;
        snprintf(cmd, sizeof(cmd), "SetHoldRegs(%d,%d,%d,%s,%s)", 
            request->index, request->addr, request->count,
            request->regs.c_str(), request->type.c_str());
        commander_->dashboardDoCmd(cmd, response->res, result);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
    catch (const std::exception& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}


bool CR5Robot::getHoldRegs(
    const std::shared_ptr<dobot_msgs::srv::GetHoldRegs::Request> request, 
    std::shared_ptr<dobot_msgs::srv::GetHoldRegs::Response> response)
{

    try
    {
        char cmd[200];
        std::vector<std::string> result;
        snprintf(cmd, sizeof(cmd), "GetHoldRegs(%d,%d,%d,%s)", 
                    request->index, request->addr, request->count,
                    request->type.c_str());
        commander_->dashboardDoCmd(cmd, response->res, result);
        for (int i =0; i<request->count; i++)
            response->regs.push_back(str2Int(result[i].c_str()));
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
    catch (const std::exception& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }


}


bool CR5Robot::continueScript(
    const std::shared_ptr<dobot_msgs::srv::ContinueScript::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ContinueScript::Response> response)
{
    try
    {
        const char* cmd = "ContinueScript()";
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::setSafeSkin(
    const std::shared_ptr<dobot_msgs::srv::SetSafeSkin::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SetSafeSkin::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetSafeSkin(%d)", request->status);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::setObstacleAvoid(
    const std::shared_ptr<dobot_msgs::srv::SetObstacleAvoid::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SetObstacleAvoid::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetObstacleAvoid(%d)", request->status);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::setCollisionLevel(
    const std::shared_ptr<dobot_msgs::srv::SetCollisionLevel::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SetCollisionLevel::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "SetCollisionLevel(%d)", request->level);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::emergencyStop(
    const std::shared_ptr<dobot_msgs::srv::EmergencyStop::Request> request, 
    std::shared_ptr<dobot_msgs::srv::EmergencyStop::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "EmergencyStop()");
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::sync(
    const std::shared_ptr<dobot_msgs::srv::Sync::Request> request, 
    std::shared_ptr<dobot_msgs::srv::Sync::Response> response)
{
    try
    {
        char result[50];
        const char* cmd = "Sync()";
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

// bool CR5Robot::getErrorID(
    // const std::shared_ptr<dobot_msgs::srv::GetErrorID::Request> request, 
    // std::shared_ptr<dobot_msgs::srv::GetErrorID::Response> response)
// {
//     dobot_msgs::srv::TcpDashboard::Request req;
//     dobot_msgs::srv::TcpDashboard::Response res;
//     req.command = "GetErrorID()";
//     if (tcpDashboard(req, res)) {
//         std::string err_vec(res.result.size(), 0);
//         size_t pos = 0;
//         for (size_t i = 0; i < err_vec.size(); ++i) {
//             if (res.result[i] == '\n' || res.result[i] == '\t') {
//                 continue;
//             }
//             err_vec[pos] = res.result[i];
//             ++pos;
//         }
// 	response->result = std::string(err_vec, 0, pos);
//         return true;
//     }
//     return false;
// }

bool CR5Robot::tcpDashboard(
    const std::shared_ptr<dobot_msgs::srv::TcpDashboard::Request> request, 
    std::shared_ptr<dobot_msgs::srv::TcpDashboard::Response> response)
{
    try
    {
        const char* cmd = request->command.c_str();
        std::vector<std::string> result;
        int res;
        commander_->dashboardDoCmd(cmd, res, result);
        for (const auto &i : result) {
            if (response->result.size() != 0) {
                response->result += ",";
            }
            response->result += i;
        }
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        return false;
    }
}

/*
 *----------------------------------------------------------------------------------------------------------------------
 *                                                  real time
 *----------------------------------------------------------------------------------------------------------------------
 */

bool CR5Robot::tcpRealData(
    const std::shared_ptr<dobot_msgs::srv::TcpRealData::Request> request, 
    std::shared_ptr<dobot_msgs::srv::TcpRealData::Response> response)
{
    if (!commander_->isConnected()) {
        return false;
    }
    uint32_t index = request->index;
    uint32_t size = request->size;
    constexpr uint32_t limit_size = sizeof(RealTimeData);
    if (index >= limit_size || index + size >= limit_size) {
        return false;
    }
    const char *data = (const char *)commander_->getRealData();
    response->real_data.insert(response->real_data.begin(), data + index, data + index + size);
    return true;
}

bool CR5Robot::movJ(
    const std::shared_ptr<dobot_msgs::srv::MovJ::Request> request, 
    std::shared_ptr<dobot_msgs::srv::MovJ::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "MovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", 
                request->x, request->y, request->z, request->a, request->b,
                request->c);
        commander_->motionDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::movL(
    const std::shared_ptr<dobot_msgs::srv::MovL::Request> request, 
    std::shared_ptr<dobot_msgs::srv::MovL::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "MovL(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", 
                    request->x, request->y, request->z, request->a, request->b, request->c);
        commander_->motionDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::servoJ(
    const std::shared_ptr<dobot_msgs::srv::ServoJ::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ServoJ::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "ServoJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", 
                    request->j1, request->j2, request->j3, request->j4, request->j5, request->j6);
        commander_->motionDoCmd(cmd, response->res);
        response->res = 0;
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::jump(
    const std::shared_ptr<dobot_msgs::srv::Jump::Request> request, 
    std::shared_ptr<dobot_msgs::srv::Jump::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Jump(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", 
                    request->offset1, request->offset2, request->offset3,
                    request->offset4, request->offset5, request->offset6);
        commander_->motionDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::arc(
    const std::shared_ptr<dobot_msgs::srv::Arc::Request> request, 
    std::shared_ptr<dobot_msgs::srv::Arc::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "Arc(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", 
                    request->x1,request->y1, request->z1, request->rx1, request->ry1, request->rz1, 
                    request->x2, request->y2, request->z2, request->rx2, request->ry2, request->rz2);
        commander_->motionDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::circle(
    const std::shared_ptr<dobot_msgs::srv::Circle::Request> request, 
    std::shared_ptr<dobot_msgs::srv::Circle::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, cmd, "Circle(%d, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f)",
                    request->count, request->x1, request->y1, request->z1, request->rx1, request->ry1, request->rz1, request->x2,
                    request->y2, request->z2, request->rx2, request->ry2, request->rz2);
        commander_->motionDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::servoP(
    const std::shared_ptr<dobot_msgs::srv::ServoP::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ServoP::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "ServoP(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", 
                    request->x, request->y, request->z, request->a, request->b, request->c);
        commander_->motionDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::relMovJTool(
    const std::shared_ptr<dobot_msgs::srv::RelMovJTool::Request> request, 
    std::shared_ptr<dobot_msgs::srv::RelMovJTool::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "RelMovJTool(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d)", 
                    request->offset1, request->offset2, request->offset3,
                    request->offset4, request->offset5, request->offset6, request->tool);
        commander_->motionDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::relMovJUser(
    const std::shared_ptr<dobot_msgs::srv::RelMovJUser::Request> request, 
    std::shared_ptr<dobot_msgs::srv::RelMovJUser::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "RelMovJUser(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d)", 
                    request->offset1, request->offset2, request->offset3,
                    request->offset4, request->offset5, request->offset6, request->user);
        commander_->motionDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::relMovLTool(
    const std::shared_ptr<dobot_msgs::srv::RelMovLTool::Request> request, 
    std::shared_ptr<dobot_msgs::srv::RelMovLTool::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "RelMovLTool(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d)", 
                    request->offset1, request->offset2, request->offset3,
                    request->offset4, request->offset5, request->offset6, request->tool);
        commander_->motionDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::relMovLUser(
    const std::shared_ptr<dobot_msgs::srv::RelMovLUser::Request> request, 
    std::shared_ptr<dobot_msgs::srv::RelMovLUser::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "RelMovLUser(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d)", 
                    request->offset1, request->offset2, request->offset3,
                    request->offset4, request->offset5, request->offset6, request->user);
        commander_->motionDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::relJointMovJ(
    const std::shared_ptr<dobot_msgs::srv::RelJointMovJ::Request> request, 
    std::shared_ptr<dobot_msgs::srv::RelJointMovJ::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "RelJointMovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", 
                    request->offset1, request->offset2, request->offset3,
                    request->offset4, request->offset5, request->offset6);
        commander_->motionDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::jointMovJ(
    const std::shared_ptr<dobot_msgs::srv::JointMovJ::Request> request, 
    std::shared_ptr<dobot_msgs::srv::JointMovJ::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "JointMovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", 
                    request->j1, request->j2, request->j3, request->j4,
                    request->j5, request->j6);
        commander_->motionDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::startTrace(
    const std::shared_ptr<dobot_msgs::srv::StartTrace::Request> request, 
    std::shared_ptr<dobot_msgs::srv::StartTrace::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "StartTrace(%s)", request->trace_name.c_str());
        commander_->motionDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::startPath(
    const std::shared_ptr<dobot_msgs::srv::StartPath::Request> request, 
    std::shared_ptr<dobot_msgs::srv::StartPath::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "StartPath(%s,%d,%d)", request->trace_name.c_str(), 
                    request->const_val, request->cart);
        commander_->motionDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::startFCTrace(
    const std::shared_ptr<dobot_msgs::srv::StartFCTrace::Request> request, 
    std::shared_ptr<dobot_msgs::srv::StartFCTrace::Response> response)
{
    try
    {
        char cmd[100];
        sprintf(cmd, "StartFCTrace(%s)", request->trace_name.c_str());
        commander_->motionDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::moveJog(
    const std::shared_ptr<dobot_msgs::srv::MoveJog::Request> request, 
    std::shared_ptr<dobot_msgs::srv::MoveJog::Response> response)
{
    try
    {
        char cmd[100];
        if (request->coord_type == 0){
            sprintf(cmd, "MoveJog(%s)", request->axis_id.c_str());
        }else{
            sprintf(cmd, "MoveJog(%s, coord_type=%d, user=%d, tool=%d)", 
                        request->axis_id.c_str(), request->coord_type, 
                        request->user, request->tool);
        }
        commander_->motionDoCmd(cmd, response->res);
        return true;
    }
    catch (const TcpClientException& err)
    {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

int CR5Robot::str2Int(const char* val)
{
    char* err;
    int mode = (int)strtol(val, &err, 10);
    if (*err != 0)
        throw std::logic_error(std::string("Invalid value : ") + val);
    return mode;
}

// void CR5Robot::backendTask(const rclcpp::TimerEvent &e) {
//     uint16_t robot_mode = commander_->getRobotMode();
//     if (robot_mode == 9 && last_robot_mode_ != 9) {
//         dobot_msgs::srv::GetErrorID::Request req;
//         dobot_msgs::srv::GetErrorID::Response res;
//         bool ok = getErrorID(req, res);
//         if (ok) {
//             RCLCPP_ERROR(this->get_logger("rclcpp"), "Robot alarm, error id %s", res->result.c_str());
//         } else {
//             RCLCPP_ERROR(this->get_logger(), "Robot alarm: %s", err.what());
//         }
//     }
//     last_robot_mode_ = robot_mode;
// }

