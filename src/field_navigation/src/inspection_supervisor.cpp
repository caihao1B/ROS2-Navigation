#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "field_navigation/cooperative_shutdown.hpp"
#include "field_navigation/supervision.hpp"

using field_navigation::Json;
using String = std_msgs::msg::String;
using Ack = std_srvs::srv::Trigger;

int main(int argc, char** argv) {
  field_navigation::initialize(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("inspection_supervisor");
  const auto config =
      ament_index_cpp::get_package_share_directory("field_navigation") +
      "/config/";
  auto config_file = node->declare_parameter<std::string>(
      "config_file", config + "inspection_policy.json");
  auto tree_file = node->declare_parameter<std::string>(
      "tree_file", config + "inspection_tree.xml");
  std::ifstream stream(config_file);
  const Json policy = Json::parse(stream);
  field_navigation::Supervisor supervisor(policy, tree_file);
  Json contract = {
      {"task_version", node->declare_parameter<std::string>(
                           "task_version", "connector-inspection")},
      {"vehicle_hash", node->declare_parameter<std::string>(
                           "vehicle_hash", "inspection-differential")},
      {"rig_revision",
       node->declare_parameter<std::string>("rig_revision", "inspection-rig")},
      {"session", node->declare_parameter<std::string>(
                      "session", "connector-inspection")}};

  Json latest = Json::object();
  auto wall = [] {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  };
  double receipt = -1, latest_stamp = -1;
  uint64_t sequence = 0;
  const auto publisher = node->create_publisher<String>("/mission/decision", 5);
  const auto subscriber = node->create_subscription<String>(
      "/mission/context", 5, [&](String::ConstSharedPtr message) {
        try {
          auto context = Json::parse(message->data);
          const double stamp = context.at("stamp").get<double>();
          const auto& sequence_value = context.at("sequence");
          if (!sequence_value.is_number_unsigned())
            throw std::runtime_error("invalid context sequence");
          const auto next = sequence_value.get<uint64_t>();
          if (!std::isfinite(stamp) || next <= sequence ||
              stamp < latest_stamp || stamp > node->now().seconds() + 0.1)
            throw std::runtime_error("invalid context clock or order");
          sequence = next;
          latest_stamp = stamp;
          receipt = wall();
          latest = std::move(context);
        } catch (const std::exception&) {
          latest = Json::object();
          receipt = -1;
        }
      });
  const auto acknowledge = node->create_service<Ack>(
      "/mission/acknowledge_manual_hold",
      [&](const std::shared_ptr<Ack::Request>,
          std::shared_ptr<Ack::Response> response) {
        response->success = supervisor.reset();
        response->message =
            response->success
                ? "Hold acknowledged"
                : "Check stopped state, observation freshness and localization";
      });
  const auto timer =
      node->create_wall_timer(std::chrono::milliseconds(30), [&] {
        const double time = node->now().seconds();
        auto context = latest;
        bool valid = true;
        for (const auto& [key, value] : contract.items()) {
          valid &= context.contains(key) && context.at(key) == value;
        }
        context["bindings_valid"] = valid;
        context["stale"] =
            receipt < 0 ||
            wall() - receipt > policy.at("context_wall_s").get<double>() ||
            time - latest_stamp > policy.at("context_age_s").get<double>() ||
            time + 0.03 < latest_stamp;
        auto result = supervisor.tick(std::move(context), time);
        result["context_sequence"] = sequence;
        result["task_version"] = contract.at("task_version");
        result["session"] = contract.at("session");
        String message;
        message.data = result.dump();
        publisher->publish(message);
      });
  field_navigation::spin(node);
  rclcpp::shutdown();
  return 0;
}
