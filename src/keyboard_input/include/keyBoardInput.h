#ifndef KEYBOARD_INPUT__KEYBOARDINPUT_H_
#define KEYBOARD_INPUT__KEYBOARDINPUT_H_

#include <chrono>
#include <termios.h>

#include <memory>
#include <string>

#include "protocol/msg/motion_servo_cmd.hpp"
#include "protocol/srv/motion_result_cmd.hpp"
#include "rclcpp/rclcpp.hpp"

class KeyboardInput
{
public:
	KeyboardInput();
	~KeyboardInput();

	void Run();

private:
	void ReadConfigParameters();
	std::string BuildNamespacedName(const std::string & resource_name) const;
	char ReadKey(double timeout_sec) const;
	void HandleKey(char key);
	void ExecuteStepCommand(float vel_x, float vel_y, float yaw_rate, const std::string & desc);
	void PublishServoCommand(bool start_frame) const;
	void PublishServoStop() const;
	void StopWalking();
	bool CallMotionResult(int32_t motion_id, const std::string & action_name);
	void ConfigureTerminal();
	void RestoreTerminal();
	void PrintHelp() const;

	rclcpp::Node::SharedPtr node_;
	rclcpp::Publisher<protocol::msg::MotionServoCmd>::SharedPtr motion_servo_pub_;
	rclcpp::Client<protocol::srv::MotionResultCmd>::SharedPtr motion_result_client_;

	std::string robot_namespace_;
	int32_t current_motion_id_;
	float current_vel_x_;
	float current_vel_y_;
	float current_yaw_rate_;
	float forward_velocity_;
	float lateral_velocity_;
	std::chrono::milliseconds step_duration_;
	std::chrono::steady_clock::time_point last_motion_input_time_;
	bool walking_active_;
	bool need_start_frame_;
	bool terminal_configured_;
	termios original_termios_;
};

#endif
