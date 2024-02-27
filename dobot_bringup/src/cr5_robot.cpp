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

#include <memory>
#include <regex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/String.h>
#include <nlohmann/json.hpp>
#include "cr5_robot.hpp"

CR5Robot::CR5Robot(std::shared_ptr<rclcpp::Node> node, std::string name)
    : Node("dobot_action_server"), node_(node), goal_{}, trajectory_duration_(1.0), last_robot_mode_(0)
{
    index_ = 0;
    memset(goal_, 0, sizeof(goal_));
    kServoJParam = std::make_shared<ServoJParam>();
}

CR5Robot::~CR5Robot()
{
    backend_task_.stop();
    RCLCPP_INFO(node_->get_logger(), "~CR5Robot");
}

void CR5Robot::init()
{
    std::string ip = node_->declare_parameter<std::string>("robot_ip_address", "192.168.5.1");

    trajectory_duration_ = node_->declare_parameter<double>("trajectory_duration", 0.3);
    RCLCPP_INFO(node_->get_logger(), "trajectory_duration : %0.2f", trajectory_duration_);

    commander_ = std::make_shared<CR5Commander>(ip);
    commander_->init();

    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::EnableRobot>(
        "dobot_bringup/srv/EnableRobot", 
        &CR5Robot::enableRobot, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::DisableRobot>(
        "dobot_bringup/srv/DisableRobot", 
        &CR5Robot::disableRobot, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ClearError>(
        "dobot_bringup/srv/ClearError", 
        &CR5Robot::clearError, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ResetRobot>(
        "dobot_bringup/srv/ResetRobot", 
        &CR5Robot::resetRobot, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SpeedFactor>(
        "dobot_bringup/srv/SpeedFactor", 
        &CR5Robot::speedFactor, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::GetErrorID>(
        "dobot_bringup/srv/GetErrorID", 
        &CR5Robot::getErrorID, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::User>(
        "dobot_bringup/srv/User", 
        &CR5Robot::user, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::Tool>(
        "dobot_bringup/srv/Tool", 
        &CR5Robot::tool, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::RobotMode>(
        "dobot_bringup/srv/RobotMode", 
        &CR5Robot::robotMode, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::PayLoad>(
        "dobot_bringup/srv/PayLoad", 
        &CR5Robot::payload, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::DO>(
        "dobot_bringup/srv/DO", 
        &CR5Robot::do, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::DOExecute>(
        "dobot_bringup/srv/DOExecute", 
        &CR5Robot::doExecute, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ToolDO>(
        "dobot_bringup/srv/ToolDO", 
        &CR5Robot::toolDO, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ToolDOExecute>(
        "dobot_bringup/srv/ToolDOExecute", 
        &CR5Robot::toolDOExecute, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::AO>(
        "dobot_bringup/srv/AO", 
        &CR5Robot::ao, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::AOExecute>(
        "dobot_bringup/srv/AOExecute", 
        &CR5Robot::aoExecute, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::AccJ>(
        "dobot_bringup/srv/AccJ", 
        &CR5Robot::accJ, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::AccL>(
        "dobot_bringup/srv/AccL", 
        &CR5Robot::accL, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SpeedJ>(
        "dobot_bringup/srv/SpeedJ", 
        &CR5Robot::speedJ, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SpeedL>(
        "dobot_bringup/srv/SpeedL", 
        &CR5Robot::speedL, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::Arch>(
        "dobot_bringup/srv/Arch", 
        &CR5Robot::arch, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::CP>(
        "dobot_bringup/srv/CP", 
        &CR5Robot::cp, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::LimZ>(
        "dobot_bringup/srv/LimZ", 
        &CR5Robot::limZ, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SetArmOrientation>(
        "dobot_bringup/srv/SetArmOrientation", 
        &CR5Robot::setArmOrientation, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SetPayload>(
        "dobot_bringup/srv/SetPayload", 
        &CR5Robot::setPayload, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::PositiveSolution>(
        "dobot_bringup/srv/PositiveSolution", 
        &CR5Robot::positiveSolution, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::InverseSolution>(
        "dobot_bringup/srv/InverseSolution", 
        &CR5Robot::inverseSolution, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::PowerOn>(
        "dobot_bringup/srv/PowerOn", 
        &CR5Robot::powerOn, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::RunScript>(
        "dobot_bringup/srv/RunScript", 
        &CR5Robot::runScript, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::StopScript>(
        "dobot_bringup/srv/StopScript", 
        &CR5Robot::stopScript, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::PauseScript>(
        "dobot_bringup/srv/PauseScript", 
        &CR5Robot::pauseScript, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ContinueScript>(
        "dobot_bringup/srv/ContinueScript", 
        &CR5Robot::continueScript, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SetSafeSkin>(
        "dobot_bringup/srv/SetSafeSkin", 
        &CR5Robot::setSafeSkin, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SetObstacleAvoid>(
        "dobot_bringup/srv/SetObstacleAvoid", 
        &CR5Robot::setObstacleAvoid, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::GetTraceStartPose>(
        "dobot_bringup/srv/GetTraceStartPose", 
        &CR5Robot::getTraceStartPose, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SetCollisionLevel>(
        "dobot_bringup/srv/SetCollisionLevel", 
        &CR5Robot::setCollisionLevel, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::GetAngle>(
        "dobot_bringup/srv/GetAngle", 
        &CR5Robot::getAngle, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::GetPose>(
        "dobot_bringup/srv/GetPose", 
        &CR5Robot::getPose, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::GetPathStartPose>(
        "dobot_bringup/srv/GetPathStartPose", 
        &CR5Robot::getPathStartPose, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::HandleTrajPoints>(
        "dobot_bringup/srv/HandleTrajPoints", 
        &CR5Robot::handleTrajPoints, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::GetSixForceData>(
        "dobot_bringup/srv/GetSixForceData", 
        &CR5Robot::getSixForceData, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SetCollideDrag>(
        "dobot_bringup/srv/SetCollideDrag", 
        &CR5Robot::setCollideDrag, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SetTerminalKeys>(
        "dobot_bringup/srv/SetTerminalKeys", 
        &CR5Robot::setTerminalKeys, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SetTerminal485>(
        "dobot_bringup/srv/SetTerminal485", 
        &CR5Robot::setTerminal485, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::GetTerminal485>(
        "dobot_bringup/srv/GetTerminal485", 
        &CR5Robot::getTerminal485, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::TCPSpeed>(
        "dobot_bringup/srv/TCPSpeed", 
        &CR5Robot::tcpSpeed, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::TCPSpeedEnd>(
        "dobot_bringup/srv/TCPSpeedEnd", 
        &CR5Robot::tcpSpeedEnd, this, std::placeholders::_1, std::placeholders::_2
    ));

    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::EmergencyStop>(
        "dobot_bringup/srv/EmergencyStop", 
        &CR5Robot::emergencyStop, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ModbusCreate>(
        "dobot_bringup/srv/ModbusCreate", 
        &CR5Robot::modbusCreate, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::MovJ>(
        "dobot_bringup/srv/MovJ", 
        &CR5Robot::movJ, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::MovL>(
        "dobot_bringup/srv/MovL", 
        &CR5Robot::movL, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::JointMovJ>(
        "dobot_bringup/srv/JointMovJ", 
        &CR5Robot::jointMovJ, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::Jump>(
        "dobot_bringup/srv/Jump", 
        &CR5Robot::jump, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::RelMovJ>(
        "dobot_bringup/srv/RelMovJ", 
        &CR5Robot::relMovJ, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::RelMovL>(
        "dobot_bringup/srv/RelMovL", 
        &CR5Robot::relMovL, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::MovLIO>(
        "dobot_bringup/srv/MovLIO", 
        &CR5Robot::movLIO, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::MovJIO>(
        "dobot_bringup/srv/MovJIO", 
        &CR5Robot::movJIO, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::Arc>(
        "dobot_bringup/srv/Arc", 
        &CR5Robot::arc, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::Circle>(
        "dobot_bringup/srv/Circle", 
        &CR5Robot::doCircle, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::RelMovJTool>(
        "dobot_bringup/srv/RelMovJTool", 
        &CR5Robot::relMovJTool, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::RelMovLTool>(
        "dobot_bringup/srv/RelMovLTool", 
        &CR5Robot::relMovLTool, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::RelMovJUser>(
        "dobot_bringup/srv/RelMovJUser", 
        &CR5Robot::relMovJUser, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::RelMovLUser>(
        "dobot_bringup/srv/RelMovLUser", 
        &CR5Robot::relMovLUser, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::RelJointMovJ>(
        "dobot_bringup/srv/RelJointMovJ", 
        &CR5Robot::relJointMovJ, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ServoJ>(
        "dobot_bringup/srv/ServoJ", 
        &CR5Robot::servoJ, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ServoP>(
        "dobot_bringup/srv/ServoP", 
        &CR5Robot::servoP, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::Sync>(
        "dobot_bringup/srv/Sync", 
        &CR5Robot::sync, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::StartTrace>(
        "dobot_bringup/srv/StartTrace", 
        &CR5Robot::startTrace, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::StartPath>(
        "dobot_bringup/srv/StartPath", 
        &CR5Robot::startPath, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::StartFCTrace>(
        "dobot_bringup/srv/StartFCTrace", 
        &CR5Robot::startFCTrace, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::MoveJog>(
        "dobot_bringup/srv/MoveJog", 
        &CR5Robot::moveJog, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::StopmoveJog>(
        "dobot_bringup/srv/StopmoveJog", 
        &CR5Robot::stopmoveJog, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::Wait>(
        "dobot_bringup/srv/Wait", 
        &CR5Robot::wait, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::Continue>(
        "dobot_bringup/srv/Continue", 
        &CR5Robot::doContinue, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::Pause>(
        "dobot_bringup/srv/Pause", 
        &CR5Robot::pause, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ModbusClose>(
        "dobot_bringup/srv/ModbusClose", 
        &CR5Robot::modbusClose, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::GetInBits>(
        "dobot_bringup/srv/GetInBits", 
        &CR5Robot::getInBits, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::GetInRegs>(
        "dobot_bringup/srv/GetInRegs", 
        &CR5Robot::getInRegs, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::GetHoldRegs>(
        "dobot_bringup/srv/GetHoldRegs", 
        &CR5Robot::getHoldRegs, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SetHoldRegs>(
        "dobot_bringup/srv/SetHoldRegs", 
        &CR5Robot::setHoldRegs, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::GetCoils>(
        "dobot_bringup/srv/GetCoils", 
        &CR5Robot::getCoils, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::SetCoils>(
        "dobot_bringup/srv/SetCoils", 
        &CR5Robot::setCoils, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ToolDI>(
        "dobot_bringup/srv/ToolDI", 
        &CR5Robot::toolDI, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::DI>(
        "dobot_bringup/srv/DI", 
        &CR5Robot::di, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ToolAI>(
        "dobot_bringup/srv/ToolAI", 
        &CR5Robot::toolAI, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::AI>(
        "dobot_bringup/srv/AI", 
        &CR5Robot::ai, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::DIGroup>(
        "dobot_bringup/srv/DIGroup", 
        &CR5Robot::diGroup, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::DOGroup>(
        "dobot_bringup/srv/DOGroup", 
        &CR5Robot::doGroup, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::BrakeControl>(
        "dobot_bringup/srv/BrakeControl", 
        &CR5Robot::brakeControl, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::StartDrag>(
        "dobot_bringup/srv/StartDrag", 
        &CR5Robot::startDrag, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::StopDrag>(
        "dobot_bringup/srv/StopDrag", 
        &CR5Robot::stopDrag, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::LoadSwitch>(
        "dobot_bringup/srv/LoadSwitch", 
        &CR5Robot::loadSwitch, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::TCPDashboard>(
        "dobot_bringup/srv/TCPDashboard", 
        &CR5Robot::tcpDashboard, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::TCPRealData>(
        "dobot_bringup/srv/TCPRealData", 
        &CR5Robot::tcpRealData, this, std::placeholders::_1, std::placeholders::_2
    ));
    server_tbl_.push_back(node_->create_service<dobot_msgs::srv::ServoJParam>(
        "dobot_bringup/srv/ServoJParam", 
        &CR5Robot::servoJParam, this, std::placeholders::_1, std::placeholders::_2
    ));

    
    //registerGoalCallback(boost::bind(&CR5Robot::goalHandle, this, _1));
    //registerCancelCallback(boost::bind(&CR5Robot::cancelHandle, this, _1));

    //backend_task_ = control_nh_.createTimer(ros::Duration(1.5), &CR5Robot::backendTask, this);

    //pubFeedInfo = control_nh_.advertise<std_msgs::String>("/dobot_bringup/msg/FeedInfo", 1000);
    //threadPubFeedBackInfo = std::thread(&CR5Robot::pubFeedBackInfo, this);
    //threadPubFeedBackInfo.detach();
    //start();
}

/*
void CR5Robot::pubFeedBackInfo()
{
    RealTimeData* realTimeData = nullptr;
    // 设置发布频率为10Hz
    ros::Rate rate(100);
    while (ros::ok()) {
        realTimeData = (const_cast<RealTimeData*>(commander_->getRealData()));
        nlohmann::json root;
        root["EnableStatus"] = realTimeData->EnableStatus;
        root["ErrorStatus"] = realTimeData->ErrorStatus;
        root["RunQueuedCmd"] = realTimeData->isRunQueuedCmd;
        std::vector<double> qActualVec;
        // memcpy(toolvectoractual.data(), realTimeData.tool_vector_actual, sizeof(realTimeData->tool_vector_actual));
        for (int i = 0; i < 6; i++) {
            qActualVec.push_back(realTimeData->q_actual[i]);
        }
        root["QactualVec"] = qActualVec;
        std::string qActualVecStr = root.dump();

        std_msgs::String msgFeedInfo;
        msgFeedInfo.data = qActualVecStr;
        pubFeedInfo.publish(msgFeedInfo);
        rate.sleep();
    }
}

void CR5Robot::feedbackHandle(const ros::TimerEvent& tm,
                              ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    control_msgs::FollowJointTrajectoryFeedback feedback;

    double current_joints[6];
    getJointState(current_joints);

    for (uint32_t i = 0; i < 6; i++) {
        feedback.joint_names.push_back(std::string("joint") + std::to_string(i + 1));
        feedback.actual.positions.push_back(current_joints[i]);
        feedback.desired.positions.push_back(goal_[i]);
    }

    handle.publishFeedback(feedback);
}

void CR5Robot::moveHandle(const ros::TimerEvent& tm,
                          ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    control_msgs::FollowJointTrajectoryGoalConstPtr trajectory = handle.getGoal();

    if (index_ < trajectory->trajectory.points.size()) {
        auto point = trajectory->trajectory.points[index_].positions;
        double tmp[6];
        for (uint32_t i = 0; i < 6; i++) {
            tmp[i] = point[i] * 180.0 / 3.1415926;
        }

        dobot_bringup::ServoJ srv;
        srv.request->offset1 = tmp[0];
        srv.request->offset2 = tmp[1];
        srv.request->offset3 = tmp[2];
        srv.request->offset4 = tmp[3];
        srv.request->offset5 = tmp[4];
        srv.request->offset6 = tmp[5];
        if (kServoJParam && kServoJParam->trajectory_duration != 0.0) {
            srv.request->t.push_back(kServoJParam->t);
            srv.request->lookahead_time.push_back(kServoJParam->lookahead_time);
            srv.request->gain.push_back(kServoJParam->gain);
            movj_timer_ = control_nh_.createTimer(ros::Duration(kServoJParam->trajectory_duration),
                                                  boost::bind(&CR5Robot::moveHandle, this, _1, handle));
        }
        servoJ(srv.request, srv.response);
        index_++;
    } else {
#define OFFSET_VAL 0.01
        double current_joints[6];
        getJointState(current_joints);
        if ((current_joints[0] >= goal_[0] - OFFSET_VAL) && (current_joints[0] <= goal_[0] + OFFSET_VAL) &&
            (current_joints[1] >= goal_[1] - OFFSET_VAL) && (current_joints[1] <= goal_[1] + OFFSET_VAL) &&
            (current_joints[2] >= goal_[2] - OFFSET_VAL) && (current_joints[2] <= goal_[2] + OFFSET_VAL) &&
            (current_joints[3] >= goal_[3] - OFFSET_VAL) && (current_joints[3] <= goal_[3] + OFFSET_VAL) &&
            (current_joints[4] >= goal_[4] - OFFSET_VAL) && (current_joints[4] <= goal_[4] + OFFSET_VAL) &&
            (current_joints[5] >= goal_[5] - OFFSET_VAL) && (current_joints[5] <= goal_[5] + OFFSET_VAL)) {
            timer_.stop();
            movj_timer_.stop();
            handle.setSucceeded();
        }
    }
}

void CR5Robot::goalHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    index_ = 0;
    for (uint32_t i = 0; i < 6; i++) {
        goal_[i] = handle.getGoal()->trajectory.points[handle.getGoal()->trajectory.points.size() - 1].positions[i];
    }
    timer_ = control_nh_.createTimer(ros::Duration(1.0), boost::bind(&CR5Robot::feedbackHandle, this, _1, handle));
    movj_timer_ = control_nh_.createTimer(ros::Duration(trajectory_duration_),
                                          boost::bind(&CR5Robot::moveHandle, this, _1, handle));
    timer_.start();
    movj_timer_.start();
    handle.setAccepted();
}

void CR5Robot::cancelHandle(ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle handle)
{
    timer_.stop();
    movj_timer_.stop();
    handle.setSucceeded();
}
*/

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

void CR5Robot::getToolVectorActual(double* val)
{
    commander_->getToolVectorActual(val);
}

/*
 *----------------------------------------------------------------------------------------------------------------------
 *                                                  dashboard
 *----------------------------------------------------------------------------------------------------------------------
 */

bool CR5Robot::enableRobot(
    const std::shared_ptr<dobot_msgs::srv::EnableRobot::Request> request, 
    std::shared_ptr<dobot_msgs::srv::EnableRobot::Response> response)
{
    try {
        char cmd[100];
        if (request->args.size() == 1) {
            sprintf(cmd, "EnableRobot(%f)", request->args[0]);
        } else if (request->args.size() == 4) {
            sprintf(cmd, "EnableRobot(%f,%f,%f,%f)", request->args[0], request->args[1], request->args[2],
                    request->args[3]);
        } else {
            sprintf(cmd, "EnableRobot()");
        }

        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const std::exception& err) {
        response->res = -1;
        return false;
    }
}

bool CR5Robot::disableRobot(
    const std::shared_ptr<dobot_msgs::srv::DisableRobot::Request> request, 
    std::shared_ptr<dobot_msgs::srv::DisableRobot::Response> response)
{
    try {
        const char* cmd = "DisableRobot()";
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const std::exception& err) {
        response->res = -1;
        return false;
    }
}

bool CR5Robot::clearError(
    const std::shared_ptr<dobot_msgs::srv::ClearError::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ClearError::Response> response)
{
    try {
        const char* cmd = "ClearError()";
        commander_->dashboardDoCmd(cmd, response->res);
        response->res = 0;
        return true;
    } catch (const std::exception& err) {
        response->res = -1;
        return false;
    }
}

bool CR5Robot::resetRobot(
    const std::shared_ptr<dobot_msgs::srv::ResetRobot::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ResetRobot::Response> response)
{
    try {
        const char* cmd = "ResetRobot()";
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::speedFactor(
    const std::shared_ptr<dobot_msgs::srv::SpeedFactor::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SpeedFactor::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "SpeedFactor(%d)", request->ratio);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::user(
    const std::shared_ptr<dobot_msgs::srv::User::Request> request, 
    std::shared_ptr<dobot_msgs::srv::User::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "User(%d)", request->index);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::tool(
    const std::shared_ptr<dobot_msgs::srv::Tool::Request> request, 
    std::shared_ptr<dobot_msgs::srv::Tool::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "Tool(%d)", request->index);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::robotMode(
    const std::shared_ptr<dobot_msgs::srv::RobotMode::Request> request, 
    std::shared_ptr<dobot_msgs::srv::RobotMode::Response> response)
{
    try {
        const char* cmd = "RobotMode()";

        std::vector<std::string> result;
        commander_->dashboardDoCmd(cmd, response->res, result);
        if (result.empty()) {
            RCLCPP_ERROR(this->get_logger(), "robotMode : Empty string");
            response->mode = -1;
            response->res = -1;
            return true;
        }
        response->mode = str2Int(result[0].c_str());
        response->res = 0;
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return true;
    } catch (const std::exception& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return true;
    }
}

bool CR5Robot::payload(
    const std::shared_ptr<dobot_msgs::srv::PayLoad::Request> request, 
    std::shared_ptr<dobot_msgs::srv::PayLoad::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "PayLoad(%0.3f,%0.3f)", request->weight, request->inertia);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::di(
    const std::shared_ptr<dobot_msgs::srv::DI::Request> request, 
    std::shared_ptr<dobot_msgs::srv::DI::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "DI(%d)", request->index);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::do(
    const std::shared_ptr<dobot_msgs::srv::DO::Request> request, 
    std::shared_ptr<dobot_msgs::srv::DO::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "DO(%d,%d)", request->index, request->status);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::doExecute(
    const std::shared_ptr<dobot_msgs::srv::DOExecute::Request> request, 
    std::shared_ptr<dobot_msgs::srv::DOExecute::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "DO(%d,%d)", request->index, request->status);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::toolDO(
    const std::shared_ptr<dobot_msgs::srv::ToolDO::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ToolDO::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "ToolDO(%d,%d)", request->index, request->status);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::toolDOExecute(
    const std::shared_ptr<dobot_msgs::srv::ToolDOExecute::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ToolDOExecute::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "ToolDOExecute(%d,%d)", request->index, request->status);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::ao(
    const std::shared_ptr<dobot_msgs::srv::AO::Request> request, 
    std::shared_ptr<dobot_msgs::srv::AO::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "AO(%d,%d)", request->index, request->status);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::aoExecute(
    const std::shared_ptr<dobot_msgs::srv::AOExecute::Request> request, 
    std::shared_ptr<dobot_msgs::srv::AOExecute::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "AO(%d,%0.3f)", request->index, static_cast<float>(request->value));
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::accJ(
    const std::shared_ptr<dobot_msgs::srv::AccJ::Request> request, 
    std::shared_ptr<dobot_msgs::srv::AccJ::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "AccJ(%d)", request->r);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::accL(
    const std::shared_ptr<dobot_msgs::srv::AccL::Request> request, 
    std::shared_ptr<dobot_msgs::srv::AccL::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "AccL(%d)", request->r);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::speedJ(
    const std::shared_ptr<dobot_msgs::srv::SpeedJ::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SpeedJ::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "SpeedJ(%d)", request->r);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::speedL(
    const std::shared_ptr<dobot_msgs::srv::SpeedL::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SpeedL::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "SpeedL(%d)", request->r);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::arch(
    const std::shared_ptr<dobot_msgs::srv::Arch::Request> request, 
    std::shared_ptr<dobot_msgs::srv::Arch::Response> response)
{
    try {
        char cmd[100];
        if (!request->cpValue.empty() && !request->cpValue[0].empty()) {
            sprintf(cmd, "Arch(%d,%s)", request->index, request->cpValue[0].c_str());

        } else {
            sprintf(cmd, "Arch(%d)", request->index);
        }
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::cp(
    const std::shared_ptr<dobot_msgs::srv::CP::Request> request, 
    std::shared_ptr<dobot_msgs::srv::CP::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "CP(%d)", request->r);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::limZ(
    const std::shared_ptr<dobot_msgs::srv::LimZ::Request> request, 
    std::shared_ptr<dobot_msgs::srv::LimZ::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "LimZ(%d)", request->value);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::setArmOrientation(
    const std::shared_ptr<dobot_msgs::srv::SetArmOrientation::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SetArmOrientation::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "SetArmOrientation(%d,%d,%d,%d)", request->LorR, request->UorD, request->ForN, request->Config6);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::setPayload(
    const std::shared_ptr<dobot_msgs::srv::SetPayload::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SetPayload::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "SetPayload(%f)", request->load);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::positiveSolution(
    const std::shared_ptr<dobot_msgs::srv::PositiveSolution::Request> request, 
    std::shared_ptr<dobot_msgs::srv::PositiveSolution::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "PositiveSolution(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%d)", request->offset1, request->offset2,
                request->offset3, request->offset4, request->offset5, request->offset6, request->user, request->tool);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::inverseSolution(
    const std::shared_ptr<dobot_msgs::srv::InverseSolution::Request> request, 
    std::shared_ptr<dobot_msgs::srv::InverseSolution::Response> response)
{
    try {
        char cmd[100];
        if (request->JointNear == "") {
            sprintf(cmd, "InverseSolution(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%d)", request->offset1, request->offset2,
                    request->offset3, request->offset4, request->offset5, request->offset6, request->user, request->tool);
        } else {
            sprintf(cmd, "InverseSolution(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%d,%d,%s)", request->offset1,
                    request->offset2, request->offset3, request->offset4, request->offset5, request->offset6, request->user,
                    request->tool, request->isJointNear, request->JointNear.c_str());
        }

        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::powerOn(
    const std::shared_ptr<dobot_msgs::srv::PowerOn::Request> request, 
    std::shared_ptr<dobot_msgs::srv::PowerOn::Response> response)
{
    try {
        const char* cmd = "PowerOn()";
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::runScript(
    const std::shared_ptr<dobot_msgs::srv::RunScript::Request> request, 
    std::shared_ptr<dobot_msgs::srv::RunScript::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "RunScript(%s)", request->projectName.c_str());
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::stopScript(
    const std::shared_ptr<dobot_msgs::srv::StopScript::Request> request, 
    std::shared_ptr<dobot_msgs::srv::StopScript::Response> response)
{
    try {
        const char* cmd = "StopScript()";
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::pauseScript(
    const std::shared_ptr<dobot_msgs::srv::PauseScript::Request> request, 
    std::shared_ptr<dobot_msgs::srv::PauseScript::Response> response)
{
    try {
        const char* cmd = "PauseScript()";
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::modbusCreate(
    const std::shared_ptr<dobot_msgs::srv::ModbusCreate::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ModbusCreate::Response> response)
{
    try {
        char cmd[300];
        std::vector<std::string> result;
        if (request->is_rtu.empty()) {
            snprintf(cmd, sizeof(cmd), "ModbusCreate(%s,%d,%d)", request->ip.c_str(), request->port, request->slave_id);
        } else {
            snprintf(cmd, sizeof(cmd), "ModbusCreate(%s,%d,%d,%d)", request->ip.c_str(), request->port, request->slave_id,
                     request->is_rtu[0]);
        }
        commander_->dashboardDoCmd(cmd, response->res, result);
        if (result.size() != 1) {
            RCLCPP_ERROR(this->get_logger(), "Haven't recv any result");
            response->res = -1;
            response->index = -1;
            return true;
        }

        response->index = str2Int(result[0].c_str());
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        response->index = -1;
        return true;
    } catch (const std::exception& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        response->index = -1;
        return true;
    }
}

bool CR5Robot::modbusClose(
    const std::shared_ptr<dobot_msgs::srv::ModbusClose::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ModbusClose::Response> response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "ModbusClose(%d)", request->index);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::getInBits(
    const std::shared_ptr<dobot_msgs::srv::GetInBits::Request> request, 
    std::shared_ptr<dobot_msgs::srv::GetInBits::Response> response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "GetInBits(%d,%d,%d)", request->index, request->addr, request->count);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::getInRegs(
    const std::shared_ptr<dobot_msgs::srv::GetInRegs::Request> request, 
    std::shared_ptr<dobot_msgs::srv::GetInRegs::Response> response)
{
    try {
        char cmd[100];
        if (!request->valType.empty() && !request->valType[0].empty()) {
            snprintf(cmd, sizeof(cmd), "GetInRegs(%d,%d,%d,%s)", request->index, request->addr, request->count,
                     request->valType[0].c_str());
        } else {
            snprintf(cmd, sizeof(cmd), "GetInRegs(%d,%d,%d)", request->index, request->addr, request->count);
        }
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::getHoldRegs(
    const std::shared_ptr<dobot_msgs::srv::GetHoldRegs::Request> request, 
    std::shared_ptr<dobot_msgs::srv::GetHoldRegs::Response> response)
{
    try {
        char cmd[100];
        std::vector<std::string> result;
        if (!request->valtype.empty() && !request->valtype[0].empty()) {
            snprintf(cmd, sizeof(cmd), "GetHoldRegs(%d,%d,%d,%s)", request->index, request->addr, request->count,
                     request->valtype[0].c_str());
        } else {
            snprintf(cmd, sizeof(cmd), "GetHoldRegs(%d,%d,%d)", request->index, request->addr, request->count);
        }

        commander_->dashboardDoCmd(cmd, response->res, result);
        if (result.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Haven't recv any result");
            response->res = -1;
            response->index = -1;
            return true;
        }
        response->index = str2Int(result[0].c_str());
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        response->index = -1;
        return false;
    }
}

bool CR5Robot::setHoldRegs(
    const std::shared_ptr<dobot_msgs::srv::SetHoldRegs::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SetHoldRegs::Response> response)
{
    try {
        char cmd[200];
        std::vector<std::string> result;
        if (!request->valtype.empty() && !request->valtype[0].empty()) {
            snprintf(cmd, sizeof(cmd), "SetHoldRegs(%d,%d,%d,%s,%s)", request->index, request->addr, request->count,
                     request->valTab.c_str(), request->valtype[0].c_str());
        } else {
            snprintf(cmd, sizeof(cmd), "SetHoldRegs(%d,%d,%d,%s)", request->index, request->addr, request->count,
                     request->valTab.c_str());
        }

        commander_->dashboardDoCmd(cmd, response->res, result);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return true;
    } catch (const std::exception& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return true;
    }
}

bool CR5Robot::getCoils(
    const std::shared_ptr<dobot_msgs::srv::GetCoils::Request> request, 
    std::shared_ptr<dobot_msgs::srv::GetCoils::Response> response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "GetCoils(%d,%d,%d)", request->index, request->addr, request->count);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::setCoils(
    const std::shared_ptr<dobot_msgs::srv::SetCoils::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SetCoils::Response> response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "SetCoils(%d,%d,%d,%s)", request->index, request->addr, request->count,
                 request->valTab.c_str());
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::getPathStartPose(
    const std::shared_ptr<dobot_msgs::srv::GetPathStartPose::Request> request, 
    std::shared_ptr<dobot_msgs::srv::GetPathStartPose::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "GetPathStartPose(%s)", request->traceName.c_str());
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::getTraceStartPose(
    const std::shared_ptr<dobot_msgs::srv::GetTraceStartPose::Request> request, 
    std::shared_ptr<dobot_msgs::srv::GetTraceStartPose::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "GetTraceStartPose(%s)", request->traceName.c_str());
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::handleTrajPoints(
    const std::shared_ptr<dobot_msgs::srv::HandleTrajPoints::Request> request, 
    std::shared_ptr<dobot_msgs::srv::HandleTrajPoints::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "HandleTrajPoints(%s)", request->traceName.c_str());
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::getSixForceData(
    const std::shared_ptr<dobot_msgs::srv::GetSixForceData::Request> request, 
    std::shared_ptr<dobot_msgs::srv::GetSixForceData::Response> response)
{
    try {
        const char* cmd = "GetSixForceData()";
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::setCollideDrag(
    const std::shared_ptr<dobot_msgs::srv::SetCollideDrag::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SetCollideDrag::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "SetCollideDrag(%d)", request->status);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::setTerminalKeys(
    const std::shared_ptr<dobot_msgs::srv::SetTerminalKeys::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SetTerminalKeys::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "SetTerminalKeys(%d)", request->status);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::setTerminal485(
    const std::shared_ptr<dobot_msgs::srv::SetTerminal485::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SetTerminal485::Response> response)
{
    try {
        char cmd[200];
        sprintf(cmd, "SetTerminal485(%d,%d,%s,%d)", request->baudRate, request->dataLen, request->parityBit.c_str(),
                request->stopBit);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::getTerminal485(
    const std::shared_ptr<dobot_msgs::srv::GetTerminal485::Request> request, 
    std::shared_ptr<dobot_msgs::srv::GetTerminal485::Response> response)
{
    try {
        const char* cmd = "GetTerminal485()";
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::tcpSpeed(
    const std::shared_ptr<dobot_msgs::srv::TCPSpeed::Request> request, 
    std::shared_ptr<dobot_msgs::srv::TCPSpeed::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "TCPSpeed(%d)", request->vt);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::tcpSpeedEnd(
    const std::shared_ptr<dobot_msgs::srv::TCPSpeedEnd::Request> request, 
    std::shared_ptr<dobot_msgs::srv::TCPSpeedEnd::Response> response)
{
    try {
        const char* cmd = "TCPSpeedEnd()";
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

/*
 *----------------------------------------------------------------------------------------------------------------------
 *                                                  real time
 *----------------------------------------------------------------------------------------------------------------------
 */

bool CR5Robot::servoJParam(
    const std::shared_ptr<dobot_msgs::srv::ServoJParam::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ServoJParam::Response> response)
{
    try {
        if (kServoJParam) {
            kServoJParam->trajectory_duration = request->trajectory_duration;
            kServoJParam->t = request->t;
            kServoJParam->lookahead_time = request->lookahead_time;
            kServoJParam->gain = request->gain;
            ROS_INFO("Set ServoJParam- trajectory_duratio: %f, t:%f, lookahead_time:%f, gain:%f ",
                     kServoJParam->trajectory_duration, kServoJParam->t, kServoJParam->lookahead_time,
                     kServoJParam->gain);
            response->res = 0;
        }
    } catch (const std::exception& err) {
        ROS_ERROR("Caught exception: %s", err.what());
        response->res = -1;
    }
    return true;
}

bool CR5Robot::tcpRealData(
    const std::shared_ptr<dobot_msgs::srv::TCPRealData::Request> request, 
    std::shared_ptr<dobot_msgs::srv::TCPRealData::Response> response)
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
    const char* data = (const char*)commander_->getRealData();
    response->real_data.insert(response->real_data.begin(), data + index, data + index + size);
    return true;
}

bool CR5Robot::toolDI(
    const std::shared_ptr<dobot_msgs::srv::ToolDI::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ToolDI::Response> response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "ToolDI(%d)", request->index);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::ai(
    const std::shared_ptr<dobot_msgs::srv::AI::Request> request, 
    std::shared_ptr<dobot_msgs::srv::AI::Response> response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "AI(%d)", request->index);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::toolAI(
    const std::shared_ptr<dobot_msgs::srv::ToolAI::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ToolAI::Response> response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "ToolAI(%d)", request->index);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::diGroup(
    const std::shared_ptr<dobot_msgs::srv::DIGroup::Request> request, 
    std::shared_ptr<dobot_msgs::srv::DIGroup::Response> response)
{
    try {
        char cmd[1000];
        std::string str{ "DIGroup(" };
        for (int i = 0; i < request->args.size(); i++) {
            if (i == request->args.size() - 1) {
                str = str + std::to_string(request->args[i]);
                break;
            }
            str = str + std::to_string(request->args[i]) + ",";
        }
        str = str + ")";
        strcpy(cmd, str.c_str());
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::doGroup(
    const std::shared_ptr<dobot_msgs::srv::DOGroup::Request> request, 
    std::shared_ptr<dobot_msgs::srv::DOGroup::Response> response)
{
    try {
        char cmd[1000];
        std::string str{ "DOGroup(" };
        for (int i = 0; i < request->args.size(); i++) {
            if (i == request->args.size() - 1) {
                str = str + std::to_string(request->args[i]);
                break;
            }
            str = str + std::to_string(request->args[i]) + ",";
        }
        str = str + ")";
        strcpy(cmd, str.c_str());
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::brakeControl(
    const std::shared_ptr<dobot_msgs::srv::BrakeControl::Request> request, 
    std::shared_ptr<dobot_msgs::srv::BrakeControl::Response> response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "BrakeControl(%d,%d)", request->axisID, request->value);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::startDrag(
    const std::shared_ptr<dobot_msgs::srv::StartDrag::Request> request, 
    std::shared_ptr<dobot_msgs::srv::StartDrag::Response> response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "StartDrag()");
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::stopDrag(
    const std::shared_ptr<dobot_msgs::srv::StopDrag::Request> request, 
    std::shared_ptr<dobot_msgs::srv::StopDrag::Response> response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "StopDrag()");
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::loadSwitch(
    const std::shared_ptr<dobot_msgs::srv::LoadSwitch::Request> request, 
    std::shared_ptr<dobot_msgs::srv::LoadSwitch::Response> response)
{
    try {
        char cmd[100];
        snprintf(cmd, sizeof(cmd), "LoadSwitch(%d)", request->status);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::continueScript(
    const std::shared_ptr<dobot_msgs::srv::ContinueScript::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ContinueScript::Response> response)
{
    try {
        const char* cmd = "ContinueScript()";
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::setSafeSkin(
    const std::shared_ptr<dobot_msgs::srv::SetSafeSkin::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SetSafeSkin::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "SetSafeSkin(%d)", request->status);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::setObstacleAvoid(
    const std::shared_ptr<dobot_msgs::srv::SetObstacleAvoid::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SetObstacleAvoid::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "SetObstacleAvoid(%d)", request->status);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::setCollisionLevel(
    const std::shared_ptr<dobot_msgs::srv::SetCollisionLevel::Request> request, 
    std::shared_ptr<dobot_msgs::srv::SetCollisionLevel::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "SetCollisionLevel(%d)", request->level);
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::getAngle(
    const std::shared_ptr<dobot_msgs::srv::GetAngle::Request> request, 
    std::shared_ptr<dobot_msgs::srv::GetAngle::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "GetAngle()");
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::getPose(
    const std::shared_ptr<dobot_msgs::srv::GetPose::Request> request, 
    std::shared_ptr<dobot_msgs::srv::GetPose::Response> response)
{
    try {
        char cmd[100];

        if (request->user.empty()) {
            sprintf(cmd, "GetPose()");
        } else {
            sprintf(cmd, "GetPose(%d,%d)", request->user[0], request->tool[0]);
        }

        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::emergencyStop(
    const std::shared_ptr<dobot_msgs::srv::EmergencyStop::Request> request, 
    std::shared_ptr<dobot_msgs::srv::EmergencyStop::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "EmergencyStop()");
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::sync(
    const std::shared_ptr<dobot_msgs::srv::Sync::Request> request, 
    std::shared_ptr<dobot_msgs::srv::Sync::Response> response)
{
    try {
        char result[50];
        const char* cmd = "Sync()";
        commander_->dashboardDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::getErrorID(
    const std::shared_ptr<dobot_msgs::srv::GetErrorID::Request> request, 
    std::shared_ptr<dobot_msgs::srv::GetErrorID::Response> response)
{
    try {
        char cmd[100];
        std::vector<std::string> result;
        sprintf(cmd, "GetErrorID()");
        commander_->dashboardDoCmd(cmd, response->res, result);
        std::string resultStr{ "" };
        for (int i = 0; i < result.size(); i++) {
            resultStr = resultStr.append(result[i]);
        }
        result.clear();
        result = regexRecv(resultStr);
        for (int i = 0; i < result.size(); i++) {
            ROS_ERROR("ErrorID: %s", result[i].c_str());
            response->errorID.push_back(std::stoi(result[i]));
        }

        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::tcpDashboard(
    const std::shared_ptr<dobot_msgs::srv::TCPDashboard::Request> request, 
    std::shared_ptr<dobot_msgs::srv::TCPDashboard::Response> response)
{
    try {
        const char* cmd = request->command.c_str();
        std::vector<std::string> result;
        int res;
        commander_->dashboardDoCmd(cmd, res, result);
        for (const auto& i : result) {
            if (response->result.size() != 0) {
                response->result += ",";
            }
            response->result += i;
        }
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        return false;
    }
}

/*
 *----------------------------------------------------------------------------------------------------------------------
 *                                                  real time
 *----------------------------------------------------------------------------------------------------------------------
 */

bool CR5Robot::movJ(
    const std::shared_ptr<dobot_msgs::srv::MovJ::Request> request, 
    std::shared_ptr<dobot_msgs::srv::MovJ::Response> response)
{
    try {
        char cmd[200];
        std::string str{ "" };
        for (int i = 0; i < request->paramValue.size(); i++) {
            str = str + "," + std::string(request->paramValue[i]);
        }
        str = str + ")";
        sprintf(cmd, "MovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request->x, request->y, request->z, request->a, request->b,
                request->c);
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::movL(
    const std::shared_ptr<dobot_msgs::srv::MovL::Request> request, 
    std::shared_ptr<dobot_msgs::srv::MovL::Response> response)
{
    try {
        char cmd[200];
        std::string str{ "" };
        for (int i = 0; i < request->paramValue.size(); i++) {
            str = str + "," + std::string(request->paramValue[i]);
        }
        str = str + ")";
        sprintf(cmd, "MovL(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request->x, request->y, request->z, request->a, request->b,
                request->c);
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::servoJ(
    const std::shared_ptr<dobot_msgs::srv::ServoJ::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ServoJ::Response> response)
{
    try {
        char cmd[100];
        if (request->t.empty()) {
            sprintf(cmd, "ServoJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request->offset1, request->offset2,
                    request->offset3, request->offset4, request->offset5, request->offset6);
        } else {
            sprintf(cmd, "ServoJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request->offset1,
                    request->offset2, request->offset3, request->offset4, request->offset5, request->offset6, request->t[0],
                    request->lookahead_time[0], request->gain[0]);
        }
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::jump(
    const std::shared_ptr<dobot_msgs::srv::Jump::Request> request, 
    std::shared_ptr<dobot_msgs::srv::Jump::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "Jump(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request->offset1, request->offset2, request->offset3,
                request->offset4, request->offset5, request->offset6);
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::arc(
    const std::shared_ptr<dobot_msgs::srv::Arc::Request> request, 
    std::shared_ptr<dobot_msgs::srv::Arc::Response> response)
{
    try {
        char cmd[200];
        sprintf(cmd, "Arc(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request->x1,
                request->y1, request->z1, request->a1, request->b1, request->c1, request->x2, request->y2, request->z2,
                request->a2, request->b2, request->c2);
        std::string str{ "" };
        for (int i = 0; i < request->paramValue.size(); i++) {
            str = str + "," + std::string(request->paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::doCircle(
    const std::shared_ptr<dobot_msgs::srv::Circle::Request> request, 
    std::shared_ptr<dobot_msgs::srv::Circle::Response> response)
{
    try {
        char cmd[200];
        sprintf(cmd, "Circle3(%d, %0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f",
                request->count, request->x1, request->y1, request->z1, request->a1, request->b1, request->c1, request->x2,
                request->y2, request->z2, request->a2, request->b2, request->c2);
        std::string str{ "" };
        for (int i = 0; i < request->paramValue.size(); i++) {
            str = str + "," + std::string(request->paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::relMovJTool(
    const std::shared_ptr<dobot_msgs::srv::RelMovJTool::Request> request, 
    std::shared_ptr<dobot_msgs::srv::RelMovJTool::Response> response)
{
    try {
        char cmd[200];
        sprintf(cmd, "RelMovJTool(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d", request->x, request->y, request->z, request->rx,
                request->ry, request->rz, request->tool);
        std::string str{ "" };
        for (int i = 0; i < request->paramValue.size(); i++) {
            str = str + "," + std::string(request->paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::relMovLTool(
    const std::shared_ptr<dobot_msgs::srv::RelMovLTool::Request> request, 
    std::shared_ptr<dobot_msgs::srv::RelMovLTool::Response> response)
{
    try {
        char cmd[200];
        sprintf(cmd, "RelMovLTool(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d", request->x, request->y, request->z, request->rx,
                request->ry, request->rz, request->tool);
        std::string str{ "" };
        for (int i = 0; i < request->paramValue.size(); i++) {
            str = str + "," + std::string(request->paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::relMovJUser(
    const std::shared_ptr<dobot_msgs::srv::RelMovJUser::Request> request, 
    std::shared_ptr<dobot_msgs::srv::RelMovJUser::Response> response)
{
    try {
        char cmd[200];
        sprintf(cmd, "RelMovJUser(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d", request->x, request->y, request->z, request->rx,
                request->ry, request->rz, request->user);
        std::string str{ "" };
        for (int i = 0; i < request->paramValue.size(); i++) {
            str = str + "," + std::string(request->paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::relMovLUser(
    const std::shared_ptr<dobot_msgs::srv::RelMovLUser::Request> request, 
    std::shared_ptr<dobot_msgs::srv::RelMovLUser::Response> response)
{
    try {
        char cmd[200];
        sprintf(cmd, "RelMovLUser(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d", request->x, request->y, request->z, request->rx,
                request->ry, request->rz, request->user);
        std::string str{ "" };
        for (int i = 0; i < request->paramValue.size(); i++) {
            str = str + "," + std::string(request->paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::relJointMovJ(
    const std::shared_ptr<dobot_msgs::srv::RelJointMovJ::Request> request, 
    std::shared_ptr<dobot_msgs::srv::RelJointMovJ::Response> response)
{
    try {
        char cmd[200];
        sprintf(cmd, "RelJointMovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request->offset1, request->offset2,
                request->offset3, request->offset4, request->offset5, request->offset6);
        std::string str{ "" };
        for (int i = 0; i < request->paramValue.size(); i++) {
            str = str + "," + std::string(request->paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::servoP(
    const std::shared_ptr<dobot_msgs::srv::ServoP::Request> request, 
    std::shared_ptr<dobot_msgs::srv::ServoP::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "ServoP(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request->x, request->y, request->z, request->a,
                request->b, request->c);
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::relMovJ(
    const std::shared_ptr<dobot_msgs::srv::RelMovJ::Request> request, 
    std::shared_ptr<dobot_msgs::srv::RelMovJ::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "RelMovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f)", request->offset1, request->offset2, request->offset3,
                request->offset4, request->offset5, request->offset6);
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::relMovL(
    const std::shared_ptr<dobot_msgs::srv::RelMovL::Request> request, 
    std::shared_ptr<dobot_msgs::srv::RelMovL::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "RelMovL(%0.3f,%0.3f,%0.3f)", request->x, request->y, request->z);
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::jointMovJ(
    const std::shared_ptr<dobot_msgs::srv::JointMovJ::Request> request, 
    std::shared_ptr<dobot_msgs::srv::JointMovJ::Response> response)
{
    try {
        char cmd[200];
        std::string str{ "" };
        for (int i = 0; i < request->paramValue.size(); i++) {
            str = str + "," + std::string(request->paramValue[i]);
        }
        str = str + ")";
        sprintf(cmd, "JointMovJ(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request->j1, request->j2, request->j3, request->j4,
                request->j5, request->j6);
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::movLIO(
    const std::shared_ptr<dobot_msgs::srv::MovLIO::Request> request, 
    std::shared_ptr<dobot_msgs::srv::MovLIO::Response> response)
{
    try {
        char cmd[200];
        sprintf(cmd, "MovLIO(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request->x, request->y, request->z, request->rx,
                request->ry, request->rz);
        std::string str{ "" };
        for (int i = 0; i < request->paramValue.size(); i++) {
            str = str + "," + std::string(request->paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::movJIO(
    const std::shared_ptr<dobot_msgs::srv::MovJIO::Request> request, 
    std::shared_ptr<dobot_msgs::srv::MovJIO::Response> response)
{
    try {
        char cmd[200];
        sprintf(cmd, "MovJIO(%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f", request->x, request->y, request->z, request->rx,
                request->ry, request->rz);
        std::string str{ "" };
        for (int i = 0; i < request->paramValue.size(); i++) {
            str = str + "," + std::string(request->paramValue[i]);
        }
        str = str + ")";
        strcat(cmd, str.c_str());
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::startTrace(
    const std::shared_ptr<dobot_msgs::srv::StartTrace::Request> request, 
    std::shared_ptr<dobot_msgs::srv::StartTrace::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "StartTrace(%s)", request->trace_name.c_str());
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::startPath(
    const std::shared_ptr<dobot_msgs::srv::StartPath::Request> request, 
    std::shared_ptr<dobot_msgs::srv::StartPath::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "StartPath(%s,%d,%d)", request->trace_name.c_str(), request->const_val, request->cart);
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::startFCTrace(
    const std::shared_ptr<dobot_msgs::srv::StartFCTrace::Request> request, 
    std::shared_ptr<dobot_msgs::srv::StartFCTrace::Response> response)
{
    try {
        char cmd[100];
        sprintf(cmd, "StartFCTrace(%s)", request->trace_name.c_str());
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::moveJog(
    const std::shared_ptr<dobot_msgs::srv::MoveJog::Request> request, 
    std::shared_ptr<dobot_msgs::srv::MoveJog::Response> response)
{
    try {
        char cmd[100];
        std::string str = "moveJog(" + std::string(request->axisID);
        for (int i = 0; i < request->paramValue.size(); i++) {
            str = str + "," + std::string(request->paramValue[i]);
        }
        str = str + ")";
        strcpy(cmd, str.c_str());
        sprintf(cmd, "MoveJog(%s)", request->axisID.c_str());
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::stopmoveJog(
    const std::shared_ptr<dobot_msgs::srv::StopmoveJog::Request> request, 
    std::shared_ptr<dobot_msgs::srv::StopmoveJog::Response> response)
{
    try {
        char cmd[100] = "moveJog()";
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::wait(
    const std::shared_ptr<dobot_msgs::srv::Wait::Request> request, 
    std::shared_ptr<dobot_msgs::srv::Wait::Response> response)
{
    try {
        char cmd[100] = "Wait()";
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::doContinue(
    const std::shared_ptr<dobot_msgs::srv::Continue::Request> request, 
    std::shared_ptr<dobot_msgs::srv::Continue::Response> response)
{
    try {
        char cmd[100] = "Continue()";
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
        RCLCPP_ERROR(this->get_logger(), err.what());
        response->res = -1;
        return false;
    }
}

bool CR5Robot::pause(
    const std::shared_ptr<dobot_msgs::srv::Pause::Request> request, 
    std::shared_ptr<dobot_msgs::srv::Pause::Response> response)
{
    try {
        char cmd[100] = "pause()";
        commander_->motionDoCmd(cmd, response->res);
        return true;
    } catch (const TcpClientException& err) {
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

/*
void CR5Robot::backendTask(const ros::TimerEvent& e)
{
    uint16_t robot_mode = commander_->getRobotMode();
    if (robot_mode == 9 && last_robot_mode_ != 9) {
        dobot_bringup::GetErrorID::Request req = {};
        dobot_bringup::GetErrorID::Response res = {};
        bool ok = getErrorID(req, res);
        if (ok) {
            for (auto errorid : res.errorID) {
                ROS_ERROR("Robot alarm, error id %d", errorid);
            }

        } else {
            ROS_ERROR("Robot alarm");
        }
    }
    last_robot_mode_ = robot_mode;
}

std::vector<std::string> CR5Robot::regexRecv(std::string getRecvInfo)
{
    std::regex pattern("-?\\d+");
    std::smatch matches;
    std::string::const_iterator searchStart(getRecvInfo.cbegin());
    std::vector<std::string> vecErrorId;
    while (std::regex_search(searchStart, getRecvInfo.cend(), matches, pattern)) {
        for (auto& match : matches) {
            vecErrorId.push_back(match.str());
        }
        searchStart = matches.suffix().first;
    }
    return vecErrorId;
};
*/