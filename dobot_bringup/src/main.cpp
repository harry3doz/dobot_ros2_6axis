/**
 ***********************************************************************************************************************
 *
 * @author ZhangRan
 * @date   2021/08/07
 *
 * <h2><center>&copy; COPYRIGHT 2021 Dobot CORPORATION</center></h2>
 *
 ***********************************************************************************************************************
 */

#include <rclcpp/rclcpp.hpp>
#include "cr5_robot.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "dobot_msgs/msg/tool_vector_actual.hpp"
#include <rclcpp/time.hpp>
#include <memory>
#include <iostream>
#include <string>
#include <cstdio>
#include <cstdlib>

using namespace std;
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto private_node = std::make_shared<rclcpp::Node>("CR5Robot");

    try
    {
        sensor_msgs::msg::JointState joint_state_msg;

        auto joint_state_pub = private_node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 100);
        dobot_msgs::msg::RobotStatus robot_status_msg;
        auto robot_status_pub = private_node->create_publisher<dobot_msgs::msg::RobotStatus>("RobotStatus", 100);

        dobot_msgs::msg::ToolVectorActual tool_vector_actual_msg;
        auto tool_vector_pub = private_node->create_publisher<dobot_msgs::msg::ToolVectorActual>("ToolVectorActual", 100);
            std::string z ="/";
            const char* robot_type = std::getenv("DOBOT_TYPE");
            std::string a = robot_type == nullptr ? "cr5" : robot_type;
            std::string b = "_robot/joint_controller/follow_joint_trajectory";
            std::string ss =  z + a+ b ;

        for (uint32_t i = 0; i < 6; i++)
        {
            joint_state_msg.position.push_back(0.0);
            joint_state_msg.name.push_back(std::string("joint") + std::to_string(i + 1));
        }

        CR5Robot robot(private_node, ss);

        double rate_value = private_node->declare_parameter("JointStatePublishRate", 10.0);

        robot.init();
        rclcpp::Rate rate(rate_value);
        double position[6];
        while (rclcpp::ok())
        {
            //
            // publish joint state
            //
            robot.getJointState(position);
            joint_state_msg.header.stamp = rclcpp::Clock().now();
            joint_state_msg.header.frame_id = "dummy_link";
            for (uint32_t i = 0; i < 6; i++)
                joint_state_msg.position[i] = position[i];
            joint_state_pub->publish(joint_state_msg);

            double val[6];
            robot.getToolVectorActual(val);
            tool_vector_actual_msg.x = val[0];
            tool_vector_actual_msg.y = val[1];
            tool_vector_actual_msg.z = val[2];
            tool_vector_actual_msg.rx = val[3];
            tool_vector_actual_msg.ry = val[4];
            tool_vector_actual_msg.rz = val[5];
            tool_vector_pub->publish(tool_vector_actual_msg);

            //
            // publish robot status
            //
            robot_status_msg.is_enable = robot.isEnable();
            // publish robots mode number here
            robot_status_msg.robot_status = robot.robotStatus();
            robot_status_msg.is_connected = robot.isConnected();

            

            robot_status_pub->publish(robot_status_msg);
            rclcpp::spin_some(private_node);
            rate.sleep();
        }
    }
    catch (const std::exception& err)
    {
        RCLCPP_ERROR(private_node->get_logger(), "%s", err.what());
        return -1;
    }

    rclcpp::shutdown();

    return 0;
}
