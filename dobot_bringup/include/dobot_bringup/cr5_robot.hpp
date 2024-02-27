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

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "dobot_msgs/srv/commander.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <dobot_msgs/srv/enable_robot.hpp>
#include <dobot_msgs/srv/disable_robot.hpp>
#include <dobot_msgs/srv/clear_error.hpp>
#include <dobot_msgs/srv/reset_robot.hpp>
#include <dobot_msgs/srv/speed_factor.hpp>
#include <dobot_msgs/srv/user.hpp>
#include <dobot_msgs/srv/tool.hpp>
#include <dobot_msgs/srv/robot_mode.hpp>
#include <dobot_msgs/srv/pay_load.hpp>
#include <dobot_msgs/srv/do.hpp>
#include <dobot_msgs/srv/do_execute.hpp>
#include <dobot_msgs/srv/tool_do.hpp>
#include <dobot_msgs/srv/tool_do_execute.hpp>
#include <dobot_msgs/srv/ao.hpp>
#include <dobot_msgs/srv/ao_execute.hpp>
#include <dobot_msgs/srv/acc_j.hpp>
#include <dobot_msgs/srv/acc_l.h>
#include <dobot_msgs/srv/speed_j.hpp>
#include <dobot_msgs/srv/speed_l.hpp>
#include <dobot_msgs/srv/arch.hpp>
#include <dobot_msgs/srv/cp.hpp>
#include <dobot_msgs/srv/lim_z.hpp>
#include <dobot_msgs/srv/set_arm_orientation.hpp>
#include <dobot_msgs/srv/power_on.hpp>
#include <dobot_msgs/srv/run_script.hpp>
#include <dobot_msgs/srv/stop_script.hpp>
#include <dobot_msgs/srv/pause_script.hpp>
#include <dobot_msgs/srv/continue_script.hpp>
#include <dobot_msgs/srv/get_hold_regs.hpp>
#include <dobot_msgs/srv/set_hold_regs.hpp>
#include <dobot_msgs/srv/set_safe_skin.hpp>
#include <dobot_msgs/srv/set_obstacle_avoid.hpp>

#include <dobot_msgs/srv/set_collision_level.hpp>
#include <dobot_msgs/srv/emergency_stop.hpp>
#include <dobot_msgs/srv/get_trace_start_pose.hpp>
#include <dobot_msgs/srv/get_path_start_pose.hpp>
#include <dobot_msgs/srv/handle_traj_points.hpp>
#include <dobot_msgs/srv/get_six_force_data.hpp>
#include <dobot_msgs/srv/set_collide_drag.hpp>
#include <dobot_msgs/srv/set_terminal_keys.hpp>
#include <dobot_msgs/srv/set_terminal485.hpp>
#include <dobot_msgs/srv/get_terminal485.hpp>
#include <dobot_msgs/srv/tcp_speed.hpp>
#include <dobot_msgs/srv/tcp_speed_end.hpp>
#include <dobot_msgs/srv/mov_j.hpp>
#include <dobot_msgs/srv/mov_l.hpp>
#include <dobot_msgs/srv/jump.hpp>
#include <dobot_msgs/srv/arc.hpp>
#include <dobot_msgs/srv/sync.hpp>
#include <dobot_msgs/srv/circle.hpp>
#include <dobot_msgs/srv/servo_j.hpp>
#include <dobot_msgs/srv/servo_p.hpp>
#include <dobot_msgs/srv/servo_j_param.hpp>
#include <dobot_msgs/srv/start_trace.hpp>
#include <dobot_msgs/srv/start_path.hpp>
#include <dobot_msgs/srv/start_fc_trace.hpp>
#include <dobot_msgs/srv/move_jog.hpp>
#include <dobot_msgs/srv/rel_mov_j.hpp>
#include <dobot_msgs/srv/rel_mov_l.hpp>
#include <dobot_msgs/srv/joint_mov_j.hpp>
#include <dobot_msgs/srv/robot_status.hpp>
#include <dobot_msgs/srv/modbus_create.hpp>
#include <dobot_msgs/srv/get_error_id.hpp>
#include <dobot_msgs/srv/set_payload.hpp>
#include <dobot_msgs/srv/positive_solution.hpp>
#include <dobot_msgs/srv/inverse_solution.hpp>
#include <dobot_msgs/srv/modbus_close.hpp>
#include <dobot_msgs/srv/get_in_bits.hpp>
#include <dobot_msgs/srv/get_in_regs.hpp>
#include <dobot_msgs/srv/get_coils.hpp>
#include <dobot_msgs/srv/set_coils.hpp>
#include <dobot_msgs/srv/di.hpp>
#include <dobot_msgs/srv/tool_di.hpp>
#include <dobot_msgs/srv/ai.hpp>
#include <dobot_msgs/srv/tool_ai.hpp>
#include <dobot_msgs/srv/di_group.hpp>
#include <dobot_msgs/srv/do_group.hpp>
#include <dobot_msgs/srv/brake_control.hpp>
#include <dobot_msgs/srv/start_drag.hpp>
#include <dobot_msgs/srv/stop_drag.hpp>
#include <dobot_msgs/srv/load_switch.hpp>
#include <dobot_msgs/srv/get_angle.hpp>
#include <dobot_msgs/srv/get_pose.hpp>
#include <dobot_msgs/srv/mov_lio.hpp>
#include <dobot_msgs/srv/mov_jio.hpp>
#include <dobot_msgs/srv/rel_mov_j_tool.hpp>
#include <dobot_msgs/srv/rel_mov_l_tool.hpp>
#include <dobot_msgs/srv/rel_mov_j_user.hpp>
#include <dobot_msgs/srv/rel_mov_l_user.hpp>
#include <dobot_msgs/srv/stopmove_jog.hpp>
#include <dobot_msgs/srv/wait.hpp>
#include <dobot_msgs/srv/continues.hpp>
#include <dobot_msgs/srv/pause.hpp>
#include <dobot_msgs/srv/rel_joint_mov_j.hpp>
#include <dobot_msgs/srv/tcp_real_data.hpp>
#include <dobot_msgs/srv/tcp_dashboard.hpp>

using namespace actionlib;
using namespace control_msgs;

struct ServoJParam
{
    double trajectory_duration = 0.0;
    double t = 0.0;
    double lookahead_time = 0.0;
    double gain = 0.0;
};

/**
 * CR5Robot
 */
class CR5Robot : public rclcpp::Node, protected rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>
{
private:
    double goal_[6];
    uint32_t index_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr movj_timer_;
    rclcpp::TimerBase::SharedPtr backend_task_;
    double trajectory_duration_;
    std::shared_ptr<rclcpp::Node> control_nh_;
    std::shared_ptr<CR5Commander> commander_;
    std::vector<rclcpp::ServiceBase::SharedPtr> server_tbl_;
    uint16_t last_robot_mode_;
    std::thread threadPubFeedBackInfo;
    auto pubFeedInfo;
    std::shared_ptr<ServoJParam> kServoJParam;

public:
    /**
     * Ctor
     * @param nh node handle
     * @param name topic
     */
    CR5Robot(std::shared_ptr<rclcpp::Node> node, std::string name);

    /**
     * CR5Robot
     */
    ~CR5Robot() override;

    /**
     * init
     */
    void init();

    /**
     * getJointState
     * @param point
     */
    void getJointState(double* point);

    /**
     * getToolVectorActual
     * @param val value
     */
    void getToolVectorActual(double* val);

    /**
     * isEnable
     * @return ture enable, otherwise false
     */
    bool isEnable() const;

    /**
     * isConnected
     * @return ture connected, otherwise false
     */
    bool isConnected() const;

protected:
    bool enableRobot(const std::shared_ptr<dobot_msgs::srv::EnableRobot::Request> request, std::shared_ptr<dobot_msgs::srv::EnableRobot::Response> response);
    bool disableRobot(const std::shared_ptr<dobot_msgs::srv::DisableRobot::Request> request, std::shared_ptr<dobot_msgs::srv::DisableRobot::Response> response);
    bool clearError(const std::shared_ptr<dobot_msgs::srv::ClearError::Request> request, std::shared_ptr<dobot_msgs::srv::ClearError::Response> response);
    bool resetRobot(const std::shared_ptr<dobot_msgs::srv::ResetRobot::Request> request, std::shared_ptr<dobot_msgs::srv::ResetRobot::Response> response);
    bool speedFactor(const std::shared_ptr<dobot_msgs::srv::SpeedFactor::Request> request, std::shared_ptr<dobot_msgs::srv::SpeedFactor::Response> response);
    bool getErrorID(const std::shared_ptr<dobot_msgs::srv::GetErrorID::Request> request, std::shared_ptr<dobot_msgs::srv::GetErrorID::Response> response);
    bool user(const std::shared_ptr<dobot_msgs::srv::User::Request> request, std::shared_ptr<dobot_msgs::srv::User::Response> response);
    bool tool(const std::shared_ptr<dobot_msgs::srv::Tool::Request> request, std::shared_ptr<dobot_msgs::srv::Tool::Response> response);
    bool robotMode(const std::shared_ptr<dobot_msgs::srv::RobotMode::Request> request, std::shared_ptr<dobot_msgs::srv::RobotMode::Response> response);
    bool payload(const std::shared_ptr<dobot_msgs::srv::PayLoad::Request> request, std::shared_ptr<dobot_msgs::srv::PayLoad::Response> response);
    bool do(const std::shared_ptr<dobot_msgs::srv::DO::Request> request, std::shared_ptr<dobot_msgs::srv::DO::Response> response);
    bool doExecute(const std::shared_ptr<dobot_msgs::srv::DOExecute::Request> request, std::shared_ptr<dobot_msgs::srv::DOExecute::Response> response);
    bool toolDO(const std::shared_ptr<dobot_msgs::srv::ToolDO::Request> request, std::shared_ptr<dobot_msgs::srv::ToolDO::Response> response);
    bool toolDOExecute(const std::shared_ptr<dobot_msgs::srv::ToolDOExecute::Request> request, std::shared_ptr<dobot_msgs::srv::ToolDOExecute::Response> response);
    bool ao(const std::shared_ptr<dobot_msgs::srv::AO::Request> request, std::shared_ptr<dobot_msgs::srv::AO::Response> response);
    bool aoExecute(const std::shared_ptr<dobot_msgs::srv::AOExecute::Request> request, std::shared_ptr<dobot_msgs::srv::AOExecute::Response> response);
    bool accJ(const std::shared_ptr<dobot_msgs::srv::AccJ::Request> request, std::shared_ptr<dobot_msgs::srv::AccJ::Response> response);
    bool accL(const std::shared_ptr<dobot_msgs::srv::AccL::Request> request, std::shared_ptr<dobot_msgs::srv::AccL::Response> response);
    bool speedJ(const std::shared_ptr<dobot_msgs::srv::SpeedJ::Request> request, std::shared_ptr<dobot_msgs::srv::SpeedJ::Response> response);
    bool speedL(const std::shared_ptr<dobot_msgs::srv::SpeedL::Request> request, std::shared_ptr<dobot_msgs::srv::SpeedL::Response> response);
    bool arch(const std::shared_ptr<dobot_msgs::srv::Arch::Request> request, std::shared_ptr<dobot_msgs::srv::Arch::Response> response);
    bool cp(const std::shared_ptr<dobot_msgs::srv::CP::Request> request, std::shared_ptr<dobot_msgs::srv::CP::Response> response);
    bool limZ(const std::shared_ptr<dobot_msgs::srv::LimZ::Request> request, std::shared_ptr<dobot_msgs::srv::LimZ::Response> response);
    bool setArmOrientation(const std::shared_ptr<dobot_msgs::srv::SetArmOrientation::Request> request, std::shared_ptr<dobot_msgs::srv::SetArmOrientation::Response> response);
    bool setPayload(const std::shared_ptr<dobot_msgs::srv::SetPayload::Request> request, std::shared_ptr<dobot_msgs::srv::SetPayload::Response> response);
    bool positiveSolution(const std::shared_ptr<dobot_msgs::srv::PositiveSolution::Request> request, std::shared_ptr<dobot_msgs::srv::PositiveSolution::Response> response);
    bool inverseSolution(const std::shared_ptr<dobot_msgs::srv::InverseSolution::Request> request, std::shared_ptr<dobot_msgs::srv::InverseSolution::Response> response);
    bool powerOn(const std::shared_ptr<dobot_msgs::srv::PowerOn::Request> request, std::shared_ptr<dobot_msgs::srv::PowerOn::Response> response);
    bool runScript(const std::shared_ptr<dobot_msgs::srv::RunScript::Request> request, std::shared_ptr<dobot_msgs::srv::RunScript::Response> response);
    bool stopScript(const std::shared_ptr<dobot_msgs::srv::StopScript::Request> request, std::shared_ptr<dobot_msgs::srv::StopScript::Response> response);
    bool pauseScript(const std::shared_ptr<dobot_msgs::srv::PauseScript::Request> request, std::shared_ptr<dobot_msgs::srv::PauseScript::Response> response);
    bool continueScript(const std::shared_ptr<dobot_msgs::srv::ContinueScript::Request> request, std::shared_ptr<dobot_msgs::srv::ContinueScript::Response> response);
    bool getHoldRegs(const std::shared_ptr<dobot_msgs::srv::GetHoldRegs::Request> request, std::shared_ptr<dobot_msgs::srv::GetHoldRegs::Response> response);
    bool setHoldRegs(const std::shared_ptr<dobot_msgs::srv::SetHoldRegs::Request> request, std::shared_ptr<dobot_msgs::srv::SetHoldRegs::Response> response);
    bool getInBits(const std::shared_ptr<dobot_msgs::srv::GetInBits::Request> request, std::shared_ptr<dobot_msgs::srv::GetInBits::Response> response);
    bool getInRegs(const std::shared_ptr<dobot_msgs::srv::GetInRegs::Request> request, std::shared_ptr<dobot_msgs::srv::GetInRegs::Response> response);
    bool getCoils(const std::shared_ptr<dobot_msgs::srv::GetCoils::Request> request, std::shared_ptr<dobot_msgs::srv::GetCoils::Response> response);
    bool setCoils(const std::shared_ptr<dobot_msgs::srv::SetCoils::Request> request, std::shared_ptr<dobot_msgs::srv::SetCoils::Response> response);
    bool di(const std::shared_ptr<dobot_msgs::srv::DI::Request> request, std::shared_ptr<dobot_msgs::srv::DI::Response> response);
    bool toolDI(const std::shared_ptr<dobot_msgs::srv::ToolDI::Request> request, std::shared_ptr<dobot_msgs::srv::ToolDI::Response> response);
    bool ai(const std::shared_ptr<dobot_msgs::srv::AI::Request> request, std::shared_ptr<dobot_msgs::srv::AI::Response> response);
    bool toolAI(const std::shared_ptr<dobot_msgs::srv::ToolAI::Request> request, std::shared_ptr<dobot_msgs::srv::ToolAI::Response> response);
    bool diGroup(const std::shared_ptr<dobot_msgs::srv::DIGroup::Request> request, std::shared_ptr<dobot_msgs::srv::DIGroup::Response> response);
    bool doGroup(const std::shared_ptr<dobot_msgs::srv::DOGroup::Request> request, std::shared_ptr<dobot_msgs::srv::DOGroup::Response> response);
    bool brakeControl(const std::shared_ptr<dobot_msgs::srv::BrakeControl::Request> request, std::shared_ptr<dobot_msgs::srv::BrakeControl::Response> response);
    bool startDrag(const std::shared_ptr<dobot_msgs::srv::StartDrag::Request> request, std::shared_ptr<dobot_msgs::srv::StartDrag::Response> response);
    bool stopDrag(const std::shared_ptr<dobot_msgs::srv::StopDrag::Request> request, std::shared_ptr<dobot_msgs::srv::StopDrag::Response> response);
    bool loadSwitch(const std::shared_ptr<dobot_msgs::srv::LoadSwitch::Request> request, std::shared_ptr<dobot_msgs::srv::LoadSwitch::Response> response);

    bool setSafeSkin(const std::shared_ptr<dobot_msgs::srv::SetSafeSkin::Request> request, std::shared_ptr<dobot_msgs::srv::SetSafeSkin::Response> response);
    bool setObstacleAvoid(const std::shared_ptr<dobot_msgs::srv::SetObstacleAvoid::Request> request, std::shared_ptr<dobot_msgs::srv::SetObstacleAvoid::Response> response);
    bool setCollisionLevel(const std::shared_ptr<dobot_msgs::srv::SetCollisionLevel::Request> request, std::shared_ptr<dobot_msgs::srv::SetCollisionLevel::Response> response);
    bool emergencyStop(const std::shared_ptr<dobot_msgs::srv::EmergencyStop::Request> request, std::shared_ptr<dobot_msgs::srv::EmergencyStop::Response> response);
    bool getTraceStartPose(const std::shared_ptr<dobot_msgs::srv::GetTraceStartPose::Request> request, std::shared_ptr<dobot_msgs::srv::GetTraceStartPose::Response> response);
    bool getPathStartPose(const std::shared_ptr<dobot_msgs::srv::GetPathStartPose::Request> request, std::shared_ptr<dobot_msgs::srv::GetPathStartPose::Response> response);
    bool handleTrajPoints(const std::shared_ptr<dobot_msgs::srv::HandleTrajPoints::Request> request, std::shared_ptr<dobot_msgs::srv::HandleTrajPoints::Response> response);
    bool getSixForceData(const std::shared_ptr<dobot_msgs::srv::GetSixForceData::Request> request, std::shared_ptr<dobot_msgs::srv::GetSixForceData::Response> response);
    bool setCollideDrag(const std::shared_ptr<dobot_msgs::srv::SetCollideDrag::Request> request, std::shared_ptr<dobot_msgs::srv::SetCollideDrag::Response> response);
    bool setTerminalKeys(const std::shared_ptr<dobot_msgs::srv::SetTerminalKeys::Request> request, std::shared_ptr<dobot_msgs::srv::SetTerminalKeys::Response> response);
    bool setTerminal485(const std::shared_ptr<dobot_msgs::srv::SetTerminal485::Request> request, std::shared_ptr<dobot_msgs::srv::SetTerminal485::Response> response);
    bool getTerminal485(const std::shared_ptr<dobot_msgs::srv::GetTerminal485::Request> request, std::shared_ptr<dobot_msgs::srv::GetTerminal485::Response> response);
    bool tcpSpeed(const std::shared_ptr<dobot_msgs::srv::TCPSpeed::Request> request, std::shared_ptr<dobot_msgs::srv::TCPSpeed::Response> response);
    bool tcpSpeedEnd(const std::shared_ptr<dobot_msgs::srv::TCPSpeedEnd::Request> request, std::shared_ptr<dobot_msgs::srv::TCPSpeedEnd::Response> response);
    bool getAngle(const std::shared_ptr<dobot_msgs::srv::GetAngle::Request> request, std::shared_ptr<dobot_msgs::srv::GetAngle::Response> response);
    bool getPose(const std::shared_ptr<dobot_msgs::srv::GetPose::Request> request, std::shared_ptr<dobot_msgs::srv::GetPose::Response> response);
    bool modbusCreate(const std::shared_ptr<dobot_msgs::srv::ModbusCreate::Request> request, std::shared_ptr<dobot_msgs::srv::ModbusCreate::Response> response);
    bool modbusClose(const std::shared_ptr<dobot_msgs::srv::ModbusClose::Request> request, std::shared_ptr<dobot_msgs::srv::ModbusClose::Response> response);
    bool movJ(const std::shared_ptr<dobot_msgs::srv::MovJ::Request> request, std::shared_ptr<dobot_msgs::srv::MovJ::Response> response);
    bool movL(const std::shared_ptr<dobot_msgs::srv::MovL::Request> request, std::shared_ptr<dobot_msgs::srv::MovL::Response> response);
    bool jointMovJ(const std::shared_ptr<dobot_msgs::srv::JointMovJ::Request> request, std::shared_ptr<dobot_msgs::srv::JointMovJ::Response> response);
    bool jump(const std::shared_ptr<dobot_msgs::srv::Jump::Request> request, std::shared_ptr<dobot_msgs::srv::Jump::Response> response);
    bool relMovJ(const std::shared_ptr<dobot_msgs::srv::RelMovJ::Request> request, std::shared_ptr<dobot_msgs::srv::RelMovJ::Response> response);
    bool relMovL(const std::shared_ptr<dobot_msgs::srv::RelMovL::Request> request, std::shared_ptr<dobot_msgs::srv::RelMovL::Response> response);
    bool movLIO(const std::shared_ptr<dobot_msgs::srv::MovLIO::Request> request, std::shared_ptr<dobot_msgs::srv::MovLIO::Response> response);
    bool movJIO(const std::shared_ptr<dobot_msgs::srv::MovJIO::Request> request, std::shared_ptr<dobot_msgs::srv::MovJIO::Response> response);
    bool arc(const std::shared_ptr<dobot_msgs::srv::Arc::Request> request, std::shared_ptr<dobot_msgs::srv::Arc::Response> response);
    bool doCircle(const std::shared_ptr<dobot_msgs::srv::Circle::Request> request, std::shared_ptr<dobot_msgs::srv::Circle::Response> response);
    bool relMovJTool(const std::shared_ptr<dobot_msgs::srv::RelMovJTool::Request> request, std::shared_ptr<dobot_msgs::srv::RelMovJTool::Response> response);
    bool relMovLTool(const std::shared_ptr<dobot_msgs::srv::RelMovLTool::Request> request, std::shared_ptr<dobot_msgs::srv::RelMovLTool::Response> response);
    bool relMovJUser(const std::shared_ptr<dobot_msgs::srv::RelMovJUser::Request> request, std::shared_ptr<dobot_msgs::srv::RelMovJUser::Response> response);
    bool relMovLUser(const std::shared_ptr<dobot_msgs::srv::RelMovLUser::Request> request, std::shared_ptr<dobot_msgs::srv::RelMovLUser::Response> response);
    bool relJointMovJ(const std::shared_ptr<dobot_msgs::srv::RelJointMovJ::Request> request, std::shared_ptr<dobot_msgs::srv::RelJointMovJ::Response> response);
    bool servoJ(const std::shared_ptr<dobot_msgs::srv::ServoJ::Request> request, std::shared_ptr<dobot_msgs::srv::ServoJ::Response> response);
    bool servoP(const std::shared_ptr<dobot_msgs::srv::ServoP::Request> request, std::shared_ptr<dobot_msgs::srv::ServoP::Response> response);
    bool sync(const std::shared_ptr<dobot_msgs::srv::Sync::Request> request, std::shared_ptr<dobot_msgs::srv::Sync::Response> response);
    bool startTrace(const std::shared_ptr<dobot_msgs::srv::StartTrace::Request> request, std::shared_ptr<dobot_msgs::srv::StartTrace::Response> response);
    bool startPath(const std::shared_ptr<dobot_msgs::srv::StartPath::Request> request, std::shared_ptr<dobot_msgs::srv::StartPath::Response> response);
    bool startFCTrace(const std::shared_ptr<dobot_msgs::srv::StartFCTrace::Request> request, std::shared_ptr<dobot_msgs::srv::StartFCTrace::Response> response);
    bool moveJog(const std::shared_ptr<dobot_msgs::srv::MoveJog::Request> request, std::shared_ptr<dobot_msgs::srv::MoveJog::Response> response);
    bool stopmoveJog(const std::shared_ptr<dobot_msgs::srv::StopmoveJog::Request> request, std::shared_ptr<dobot_msgs::srv::StopmoveJog::Response> response);
    bool wait(const std::shared_ptr<dobot_msgs::srv::Wait::Request> request, std::shared_ptr<dobot_msgs::srv::Wait::Response> response);
    bool doContinue(const std::shared_ptr<dobot_msgs::srv::Continue::Request> request, std::shared_ptr<dobot_msgs::srv::Continue::Response> response);
    bool pause(const std::shared_ptr<dobot_msgs::srv::Pause::Request> request, std::shared_ptr<dobot_msgs::srv::Pause::Response> response);
    bool tcpRealData(const std::shared_ptr<dobot_msgs::srv::TCPRealData::Request> request, std::shared_ptr<dobot_msgs::srv::TCPRealData::Response> response);
    bool tcpDashboard(const std::shared_ptr<dobot_msgs::srv::TCPDashboard::Request> request, std::shared_ptr<dobot_msgs::srv::TCPDashboard::Response> response);
    bool servoJParam(const std::shared_ptr<dobot_msgs::srv::ServoJParam::Request> request, std::shared_ptr<dobot_msgs::srv::ServoJParam::Response> response);

private:
    static int str2Int(const char* val);

    //void feedbackHandle(const ros::TimerEvent& tm,
    //                    actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    //void moveHandle(const ros::TimerEvent& tm, actionlib::ActionServer<FollowJointTrajectoryAction>::GoalHandle handle);
    rclcpp_action::GoalResponse goalHandle(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> handle,
        const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal 
    );
    rclcpp_action::CancelResponse cancelHandle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> handle);

    //void backendTask(const ros::TimerEvent& e);
    //void pubFeedBackInfo();
    //std::vector<std::string> regexRecv(std::string getRecvInfo);
};
