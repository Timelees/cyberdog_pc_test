#include "keyBoardInput.h"

#include <sys/select.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <limits.h>
#include <thread>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace
{
constexpr int32_t kMotionIdGetDown = 101;
constexpr int32_t kMotionIdRecoveryStand = 111;
constexpr int32_t kMotionIdSlowWalk = 303;
constexpr int32_t kMotionIdFastWalk = 308;

constexpr double kLoopPeriodSec = 0.05;
constexpr int64_t kDefaultStepDurationMs = 250;
constexpr float kForwardVelocity = 0.20F;
constexpr float kLateralVelocity = 0.15F;
constexpr float kTurnYawRate = 1.0F;
constexpr float kStepHeight = 0.05F;
constexpr int32_t kServoValue = 2;

std::string Trim(const std::string & value)
{
	const auto first = value.find_first_not_of(" \t\r\n");
	if (first == std::string::npos) {
		return "";
	}

	const auto last = value.find_last_not_of(" \t\r\n");
	return value.substr(first, last - first + 1);
}

std::string TrimQuotes(const std::string & value)
{
	if (value.size() >= 2 &&
		((value.front() == '"' && value.back() == '"') ||
		(value.front() == '\'' && value.back() == '\'')))
	{
		return value.substr(1, value.size() - 2);
	}
	return value;
}

std::string NormalizeNamespace(const std::string & raw_namespace)
{
	std::string name = Trim(raw_namespace);
	while (!name.empty() && name.front() == '/') {
		name.erase(name.begin());
	}
	while (!name.empty() && name.back() == '/') {
		name.pop_back();
	}
	return name;
}

std::string ReadNamespaceFromConfigFile(const std::string & file_path)
{
	std::ifstream input(file_path);
	if (!input.is_open()) {
		return "";
	}

	std::string line;
	while (std::getline(input, line)) {
		const auto comment_pos = line.find('#');
		if (comment_pos != std::string::npos) {
			line = line.substr(0, comment_pos);
		}

		const auto colon_pos = line.find(':');
		if (colon_pos == std::string::npos) {
			continue;
		}

		const std::string key = Trim(line.substr(0, colon_pos));
		if (key != "robot_namespace" && key != "namespace") {
			continue;
		}

		const std::string value = TrimQuotes(Trim(line.substr(colon_pos + 1)));
		return NormalizeNamespace(value);
	}

	return "";
}
}  // namespace

KeyboardInput::KeyboardInput()
: node_(rclcpp::Node::make_shared("keyboard_input")),
	robot_namespace_("cyberdog_1"),
	current_motion_id_(kMotionIdSlowWalk),
	current_vel_x_(0.0F),
	current_vel_y_(0.0F),
	current_yaw_rate_(0.0F),
	forward_velocity_(kForwardVelocity),
	lateral_velocity_(kLateralVelocity),
	step_duration_(kDefaultStepDurationMs),
	last_motion_input_time_(std::chrono::steady_clock::now()),
	walking_active_(false),
	need_start_frame_(false),
	terminal_configured_(false)
{
	ReadConfigParameters();

	motion_servo_pub_ = node_->create_publisher<protocol::msg::MotionServoCmd>(
		BuildNamespacedName("motion_servo_cmd"), rclcpp::SystemDefaultsQoS());
	motion_result_client_ = node_->create_client<protocol::srv::MotionResultCmd>(
		BuildNamespacedName("motion_result_cmd"));

	RCLCPP_INFO(
		node_->get_logger(),
		"加载控制配置: namespace=/%s, step_duration=%ldms",
		robot_namespace_.c_str(), step_duration_.count());
	ConfigureTerminal();
}

KeyboardInput::~KeyboardInput()
{
	RestoreTerminal();
}

void KeyboardInput::Run()
{
	PrintHelp();

	while (rclcpp::ok()) {
		rclcpp::spin_some(node_);

		const char key = ReadKey(kLoopPeriodSec);
		if (key != '\0') {
			if (key == 'q' || key == 'Q') {
				break;
			}
			HandleKey(key);
		}

		if (walking_active_) {
			const auto now = std::chrono::steady_clock::now();
			if (now - last_motion_input_time_ >= step_duration_) {
				StopWalking();
			} else {
				PublishServoCommand(need_start_frame_);
				need_start_frame_ = false;
			}
		}
	}

	StopWalking();
}

void KeyboardInput::ReadConfigParameters()
{
	node_->declare_parameter<std::string>("namespace", "");
	node_->declare_parameter<std::string>("robot_namespace", robot_namespace_);
	node_->declare_parameter<int64_t>("step_duration_ms", kDefaultStepDurationMs);
	node_->declare_parameter<double>("forward_velocity", static_cast<double>(kForwardVelocity));
	node_->declare_parameter<double>("lateral_velocity", static_cast<double>(kLateralVelocity));
	node_->declare_parameter<std::string>("namespace_config_file", "");

	const std::string declared_default_namespace = robot_namespace_;
	const std::string namespace_from_alias_param =
		NormalizeNamespace(node_->get_parameter("namespace").as_string());
	const std::string namespace_from_param =
		NormalizeNamespace(node_->get_parameter("robot_namespace").as_string());

	std::vector<std::string> candidate_files;
	const std::string namespace_config_file =
		node_->get_parameter("namespace_config_file").as_string();
	if (!namespace_config_file.empty()) {
		candidate_files.push_back(namespace_config_file);
	}

	char cwd_buffer[PATH_MAX] = {0};
	if (getcwd(cwd_buffer, sizeof(cwd_buffer)) != nullptr) {
		const std::string cwd(cwd_buffer);
		candidate_files.push_back(cwd + "/src/config/namespace_config.yaml");
		candidate_files.push_back(cwd + "/config/namespace_config.yaml");
	}

	try {
		candidate_files.push_back(
			ament_index_cpp::get_package_share_directory("keyboard_input") +
			"/config/namespace_config.yaml");
	} catch (const std::exception &) {
		
	}

	for (const auto & candidate : candidate_files) {
		const std::string file_namespace = ReadNamespaceFromConfigFile(candidate);
		if (!file_namespace.empty()) {
			robot_namespace_ = file_namespace;
			break;
		}
	}

	if (!namespace_from_param.empty() && namespace_from_param != declared_default_namespace) {
		robot_namespace_ = namespace_from_param;
	}

	if (!namespace_from_alias_param.empty()) {
		robot_namespace_ = namespace_from_alias_param;
	}

	if (robot_namespace_.empty()) {
		robot_namespace_ = "cyberdog_1";
	}

	step_duration_ = std::chrono::milliseconds(std::max<int64_t>(
		50, node_->get_parameter("step_duration_ms").as_int()));
	forward_velocity_ = static_cast<float>(node_->get_parameter("forward_velocity").as_double());
	lateral_velocity_ = static_cast<float>(node_->get_parameter("lateral_velocity").as_double());
}

std::string KeyboardInput::BuildNamespacedName(const std::string & resource_name) const
{
	if (robot_namespace_.empty()) {
		return "/" + resource_name;
	}
	return "/" + robot_namespace_ + "/" + resource_name;
}

char KeyboardInput::ReadKey(double timeout_sec) const
{
	if (!terminal_configured_) {
		return '\0';
	}

	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(STDIN_FILENO, &readfds);

	timeval timeout;
	timeout.tv_sec = static_cast<long>(timeout_sec);
	timeout.tv_usec = static_cast<long>((timeout_sec - timeout.tv_sec) * 1000000.0);

	const int ready = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);
	if (ready <= 0) {
		return '\0';
	}

	char key = '\0';
	bool saw_space = false;

	while (true) {
		char current = '\0';
		if (read(STDIN_FILENO, &current, 1) <= 0) {
			break;
		}

		if (current == ' ') {
			saw_space = true;
		}
		key = current;

		fd_set pending_readfds;
		FD_ZERO(&pending_readfds);
		FD_SET(STDIN_FILENO, &pending_readfds);

		timeval pending_timeout;
		pending_timeout.tv_sec = 0;
		pending_timeout.tv_usec = 0;

		const int pending_ready = select(
			STDIN_FILENO + 1, &pending_readfds, nullptr, nullptr, &pending_timeout);
		if (pending_ready <= 0) {
			break;
		}
	}

	return saw_space ? ' ' : key;
}

void KeyboardInput::HandleKey(char key)
{
	switch (key) {
		case '1':
			StopWalking();
			CallMotionResult(kMotionIdGetDown, "趴下");
			break;

		case '2':
			StopWalking();
			CallMotionResult(kMotionIdRecoveryStand, "站立");
			break;

		case '3':
			current_motion_id_ = kMotionIdSlowWalk;
			RCLCPP_INFO(node_->get_logger(), "切换为慢走步态");
			break;

		case '4':
			current_motion_id_ = kMotionIdFastWalk;
			RCLCPP_INFO(node_->get_logger(), "切换为快走步态");
			break;

		case 'w':
		case 'W':
			ExecuteStepCommand(forward_velocity_, 0.0F, 0.0F, "前进");
			break;

		case 's':
		case 'S':
			ExecuteStepCommand(-forward_velocity_, 0.0F, 0.0F, "后退");
			break;

		case 'a':
		case 'A':
			// 机器人机体坐标系中，y 正方向约定为左侧，因此左移使用正 y。
			ExecuteStepCommand(0.0F, lateral_velocity_, 0.0F, "左移");
			break;

		case 'd':
		case 'D':
			ExecuteStepCommand(0.0F, -lateral_velocity_, 0.0F, "右移");
			break;

		case 'j':
		case 'J':
			ExecuteStepCommand(0.0F, 0.0F, kTurnYawRate, "左转");
			break;

		case 'l':
		case 'L':
			ExecuteStepCommand(0.0F, 0.0F, -kTurnYawRate, "右转");
			break;

		case ' ':
			StopWalking();
			RCLCPP_INFO(node_->get_logger(), "发送停止指令");
			break;

		case 'h':
		case 'H':
			PrintHelp();
			break;

		default:
			break;
	}
}

void KeyboardInput::ExecuteStepCommand(
	float vel_x,
	float vel_y,
	float yaw_rate,
	const std::string & desc)
{
	const bool command_changed =
		!walking_active_ ||
		current_vel_x_ != vel_x ||
		current_vel_y_ != vel_y ||
		current_yaw_rate_ != yaw_rate;

	current_vel_x_ = vel_x;
	current_vel_y_ = vel_y;
	current_yaw_rate_ = yaw_rate;
	last_motion_input_time_ = std::chrono::steady_clock::now();

	// 持续按住按键时依赖重复键盘事件续命；松开超过 step_duration_ 后自动停止。
	if (!walking_active_) {
		need_start_frame_ = true;
	}
	walking_active_ = true;

	if (command_changed) {
		RCLCPP_INFO(
			node_->get_logger(),
			"%s: motion_id=%d, vel_des=[%.2f, %.2f, %.2f], 松键超时=%ldms",
			desc.c_str(), current_motion_id_, current_vel_x_, current_vel_y_, current_yaw_rate_,
			step_duration_.count());
	}
}

void KeyboardInput::PublishServoCommand(bool start_frame) const
{
	protocol::msg::MotionServoCmd msg;
	msg.motion_id = current_motion_id_;
	msg.cmd_type = start_frame ?
		protocol::msg::MotionServoCmd::SERVO_START :
		protocol::msg::MotionServoCmd::SERVO_DATA;
	msg.cmd_source = protocol::msg::MotionServoCmd::APP;
	msg.value = kServoValue;

	// 按底层消息的固定维度补零，避免进入 MotionAction 时触发尺寸不匹配日志。
	msg.vel_des = std::vector<float>{current_vel_x_, current_vel_y_, current_yaw_rate_};
	msg.rpy_des = std::vector<float>{0.0F, 0.0F, 0.0F};
	msg.pos_des = std::vector<float>{0.0F, 0.0F, 0.0F};
	msg.acc_des = std::vector<float>{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
	msg.ctrl_point = std::vector<float>{0.0F, 0.0F, 0.0F};
	msg.foot_pose = std::vector<float>{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
	msg.step_height = std::vector<float>{kStepHeight, kStepHeight};

	motion_servo_pub_->publish(msg);
}

void KeyboardInput::PublishServoStop() const
{
	protocol::msg::MotionServoCmd msg;
	msg.motion_id = current_motion_id_;
	msg.cmd_type = protocol::msg::MotionServoCmd::SERVO_END;
	msg.cmd_source = protocol::msg::MotionServoCmd::APP;
	msg.value = kServoValue;
	msg.vel_des = std::vector<float>{0.0F, 0.0F, 0.0F};
	msg.rpy_des = std::vector<float>{0.0F, 0.0F, 0.0F};
	msg.pos_des = std::vector<float>{0.0F, 0.0F, 0.0F};
	msg.acc_des = std::vector<float>{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
	msg.ctrl_point = std::vector<float>{0.0F, 0.0F, 0.0F};
	msg.foot_pose = std::vector<float>{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
	msg.step_height = std::vector<float>{kStepHeight, kStepHeight};
	motion_servo_pub_->publish(msg);
}

void KeyboardInput::StopWalking()
{
	if (walking_active_) {
		PublishServoStop();

		// 给底层一点时间处理结束帧，避免紧接着发送结果指令时状态仍处于 ServoCmd。
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	current_vel_x_ = 0.0F;
	current_vel_y_ = 0.0F;
	current_yaw_rate_ = 0.0F;
	last_motion_input_time_ = std::chrono::steady_clock::now();
	walking_active_ = false;
	need_start_frame_ = false;
}

bool KeyboardInput::CallMotionResult(int32_t motion_id, const std::string & action_name)
{
	if (!motion_result_client_->wait_for_service(std::chrono::seconds(2))) {
		RCLCPP_ERROR(node_->get_logger(), "服务 motion_result_cmd 不可用，无法执行%s", action_name.c_str());
		return false;
	}

	auto request = std::make_shared<protocol::srv::MotionResultCmd::Request>();
	request->motion_id = motion_id;
	request->cmd_source = protocol::srv::MotionResultCmd::Request::APP;
	request->vel_des = std::vector<float>{0.0F, 0.0F, 0.0F};
	request->rpy_des = std::vector<float>{0.0F, 0.0F, 0.0F};
	request->pos_des = std::vector<float>{0.0F, 0.0F, 0.0F};
	request->acc_des = std::vector<float>{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
	request->ctrl_point = std::vector<float>{0.0F, 0.0F, 0.0F};
	request->foot_pose = std::vector<float>{0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
	request->step_height = std::vector<float>{0.0F, 0.0F};
	request->duration = 0;
	request->value = 0;
	request->contact = 0;

	const auto future = motion_result_client_->async_send_request(request);
	const auto ret = rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5));
	if (ret != rclcpp::FutureReturnCode::SUCCESS) {
		RCLCPP_ERROR(node_->get_logger(), "%s指令调用超时或失败", action_name.c_str());
		return false;
	}

	const auto response = future.get();
	if (!response->result) {
		RCLCPP_ERROR(
			node_->get_logger(), "%s执行失败，motion_id=%d, code=%d",
			action_name.c_str(), motion_id, response->code);
		return false;
	}

	RCLCPP_INFO(
		node_->get_logger(), "%s执行成功，motion_id=%d, code=%d",
		action_name.c_str(), motion_id, response->code);
	return true;
}

void KeyboardInput::ConfigureTerminal()
{
	if (!isatty(STDIN_FILENO)) {
		RCLCPP_WARN(node_->get_logger(), "当前输入不是终端，无法读取实时键盘输入");
		return;
	}

	if (tcgetattr(STDIN_FILENO, &original_termios_) != 0) {
		RCLCPP_ERROR(node_->get_logger(), "读取终端属性失败");
		return;
	}

	termios raw = original_termios_;
	raw.c_lflag &= static_cast<unsigned long>(~(ICANON | ECHO));
	raw.c_cc[VMIN] = 0;
	raw.c_cc[VTIME] = 0;

	if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
		RCLCPP_ERROR(node_->get_logger(), "设置终端原始输入模式失败");
		return;
	}

	terminal_configured_ = true;
}

void KeyboardInput::RestoreTerminal()
{
	if (!terminal_configured_) {
		return;
	}

	tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
	terminal_configured_ = false;
}

void KeyboardInput::PrintHelp() const
{
	std::printf(
		"\n"
		"Cyberdog2 键盘控制说明\n"
		"=======================\n"
		"1 : 站立切换到趴下\n"
		"2 : 趴下切换到站立\n"
		"3 : 慢走步态\n"
		"4 : 快走步态\n"
		"w : 前进\n"
		"s : 后退\n"
		"a : 左移\n"
		"d : 右移\n"
		"j : 左转\n"
		"l : 右转\n"
		"空格 : 停止\n"
		"h : 再次打印帮助\n"
		"q : 退出程序\n\n");
}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	KeyboardInput keyboard_input;
	keyboard_input.Run();
	rclcpp::shutdown();
	return 0;
}
