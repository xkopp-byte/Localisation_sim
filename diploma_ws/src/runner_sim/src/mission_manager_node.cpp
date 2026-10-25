#include <algorithm>
#include <cctype>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <runner_sim/msg/mission_command.hpp>
#include <runner_sim/msg/mission_status.hpp>

namespace
{
struct MissionRow
{
  int command_id{0};
  int runner_number{0};
  double x{0.0};
  double y{0.0};
  double circle_radius_m{0.0};
  double speed_percent{0.0};
  bool hard_point{true};
  // Optional: time to wait (sim time) after the row becomes current before it is sent.
  double start_delay_s{0.0};
};

std::string trim(std::string value)
{
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), [](unsigned char character) {
    return !std::isspace(character);
  }));
  value.erase(std::find_if(value.rbegin(), value.rend(), [](unsigned char character) {
    return !std::isspace(character);
  }).base(), value.end());
  return value;
}

std::vector<std::string> split_csv(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(trim(field));
  }
  return fields;
}

bool parse_bool(const std::string & value)
{
  const std::string lower = [&value]() {
    std::string result = value;
    std::transform(result.begin(), result.end(), result.begin(), [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
    return result;
  }();
  if (lower == "true" || lower == "1" || lower == "hard") {
    return true;
  }
  if (lower == "false" || lower == "0" || lower == "soft") {
    return false;
  }
  throw std::runtime_error("hard_point must be true, false, 1, 0, hard, or soft");
}
}  // namespace

class MissionManagerNode : public rclcpp::Node
{
public:
  MissionManagerNode()
  : Node("mission_manager_node")
  {
    mission_file_ = declare_parameter<std::string>("mission_file", "");
    max_runner_number_ = declare_parameter<int>("max_runner_number", 0);
    publish_period_ms_ = declare_parameter<int>("publish_period_ms", 200);
    if (mission_file_.empty()) {
      throw std::runtime_error("mission_file parameter is required");
    }

    load_mission();
    for (const auto & [runner_number, rows] : missions_) {
      publishers_[runner_number] = create_publisher<runner_sim::msg::MissionCommand>(
        "/runner_" + two_digits(runner_number) + "/mission_command", 10);
      status_subscriptions_[runner_number] = create_subscription<runner_sim::msg::MissionStatus>(
        "/runner_" + two_digits(runner_number) + "/mission_status", 10,
        [this, runner_number](const runner_sim::msg::MissionStatus::SharedPtr message) {
          status_callback(runner_number, message);
        });
      next_row_[runner_number] = 0;
      publish_current_command(runner_number);
    }

    publish_timer_ = create_wall_timer(
      std::chrono::milliseconds(publish_period_ms_),
      std::bind(&MissionManagerNode::republish_commands, this));
    RCLCPP_INFO(get_logger(), "Loaded %zu runner mission queues from %s",
      missions_.size(), mission_file_.c_str());
  }

private:
  static std::string two_digits(int runner_number)
  {
    char buffer[3];
    std::snprintf(buffer, sizeof(buffer), "%02d", runner_number);
    return buffer;
  }

  void load_mission()
  {
    std::ifstream input(mission_file_);
    if (!input.is_open()) {
      throw std::runtime_error("Cannot open mission file: " + mission_file_);
    }

    std::string line;
    int line_number = 0;
    int command_id = 0;
    while (std::getline(input, line)) {
      ++line_number;
      line = trim(line);
      if (line.empty() || line.front() == '#') {
        continue;
      }
      const auto fields = split_csv(line);
      if (!fields.empty() && fields.front() == "runner_number") {
        continue;
      }
      if (fields.size() != 6 && fields.size() != 7) {
        throw std::runtime_error(
          "Mission line " + std::to_string(line_number) + " must contain 6 or 7 CSV fields");
      }

      try {
        MissionRow row;
        row.command_id = ++command_id;
        row.runner_number = std::stoi(fields[0]);
        row.x = std::stod(fields[1]);
        row.y = std::stod(fields[2]);
        row.circle_radius_m = std::stod(fields[3]);
        row.speed_percent = std::stod(fields[4]);
        row.hard_point = parse_bool(fields[5]);
        if (fields.size() == 7) {
          row.start_delay_s = std::stod(fields[6]);
        }
        if (row.runner_number < 1 || (max_runner_number_ > 0 &&
          row.runner_number > max_runner_number_) || row.speed_percent < 0.0 ||
          row.speed_percent > 100.0 || row.start_delay_s < 0.0)
        {
          throw std::runtime_error(
            "runner_number, speed_percent, or start_delay_s out of range");
        }
        missions_[row.runner_number].push_back(row);
      } catch (const std::exception & exception) {
        throw std::runtime_error(
          "Invalid mission line " + std::to_string(line_number) + ": " + exception.what());
      }
    }
    if (missions_.empty()) {
      throw std::runtime_error("Mission file contains no commands");
    }
  }

  void publish_current_command(int runner_number)
  {
    auto & rows = missions_.at(runner_number);
    auto & index = next_row_.at(runner_number);
    if (index >= rows.size() || completed_[runner_number]) {
      return;
    }
    const MissionRow & row = rows[index];
    if (row.start_delay_s > 0.0) {
      const rclcpp::Time now = get_clock()->now();
      if (now.nanoseconds() == 0) {
        return;  // Simulation clock not received yet.
      }
      auto activation = row_activation_time_.find(runner_number);
      if (activation == row_activation_time_.end()) {
        activation = row_activation_time_.emplace(runner_number, now).first;
      }
      if ((now - activation->second).seconds() < row.start_delay_s) {
        return;
      }
    }
    runner_sim::msg::MissionCommand message;
    message.command_id = row.command_id;
    message.runner_number = row.runner_number;
    message.x = row.x;
    message.y = row.y;
    message.circle_radius_m = row.circle_radius_m;
    message.speed_percent = row.speed_percent;
    message.hard_point = row.hard_point;
    publishers_.at(runner_number)->publish(message);
  }

  void republish_commands()
  {
    for (const auto & [runner_number, rows] : missions_) {
      (void)rows;
      publish_current_command(runner_number);
    }
  }

  void status_callback(
    int runner_number, const runner_sim::msg::MissionStatus::SharedPtr & status)
  {
    if (!status->valid) {
      RCLCPP_ERROR(get_logger(), "Runner %02d rejected command: %s",
        runner_number, status->error.c_str());
      completed_[runner_number] = true;
      return;
    }
    if (!status->reached || completed_[runner_number]) {
      return;
    }
    auto & rows = missions_.at(runner_number);
    auto & index = next_row_.at(runner_number);
    if (index >= rows.size() || status->command_id != rows[index].command_id) {
      return;
    }
    ++index;
    row_activation_time_.erase(runner_number);
    if (index == rows.size()) {
      completed_[runner_number] = true;
      RCLCPP_INFO(get_logger(), "Runner %02d completed its mission", runner_number);
    } else {
      publish_current_command(runner_number);
    }
  }

  std::string mission_file_;
  int max_runner_number_{0};
  int publish_period_ms_{200};
  std::map<int, std::vector<MissionRow>> missions_;
  std::map<int, std::size_t> next_row_;
  std::map<int, bool> completed_;
  std::map<int, rclcpp::Time> row_activation_time_;
  std::map<int, rclcpp::Publisher<runner_sim::msg::MissionCommand>::SharedPtr> publishers_;
  std::map<int, rclcpp::Subscription<runner_sim::msg::MissionStatus>::SharedPtr> status_subscriptions_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MissionManagerNode>());
  } catch (const std::exception & exception) {
    fprintf(stderr, "mission_manager_node: %s\n", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
