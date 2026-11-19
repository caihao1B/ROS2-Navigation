#pragma once
#include "field_navigation/policy.hpp"

namespace field_navigation {

// Nav2 cancellation is asynchronous. A policy preference becomes a motion
// directive only after navigation, chassis and tool have acknowledged a stop.
class Supervisor {
 public:
  Supervisor(const Json& thresholds, const std::string& tree_xml)
      : policy_(thresholds) {
    policy_.register_nodes(factory_);
    tree_ = factory_.createTreeFromFile(tree_xml);
  }

  Json tick(Json observation, double time) {
    policy_.prepare(std::move(observation), time);
    tree_.tickOnce();
    auto result = policy_.decision;
    if (result.at("reason_code") == "LOCALIZATION_ADMISSION_WAIT" &&
        policy_.flag("localized") && !policy_.flag("sensors_healthy")) {
      result["reason_code"] = "PERCEPTION_ADMISSION_WAIT";
    }

    const auto selected = result.at("action").get<std::string>();
    auto command = selected;
    const bool stationary_action = selected == "WAIT" || selected == "REPLAN" ||
                                   selected == "DEFER" || selected == "HUMAN" ||
                                   selected == "COMPLETE";
    const bool stopped = policy_.flag("nav_idle") &&
                         policy_.flag("tool_stopped") &&
                         policy_.flag("stopped");
    const bool speed_change_only =
        (selected == "FOLLOW" && active_ == "EXECUTE") ||
        (selected == "EXECUTE" && active_ == "FOLLOW");
    const bool transition = selected != active_ && !speed_change_only;

    if (selected != "STOP" && !stopped &&
        (stationary_action || transition || cancel_since_ >= 0)) {
      if (cancel_since_ < 0) cancel_since_ = time;
      command = "CANCEL";
      result["pending_action"] = selected;
      result["cancel_request"] = true;
      if (time - cancel_since_ >
          policy_.thresholds.at("cancel_confirm_s").get<double>()) {
        policy_.manual = true;
        policy_.manual_reason = "CANCEL_OR_STOP_CONFIRMATION_TIMEOUT";
        result["reason_code"] = policy_.manual_reason;
        command = "STOP";
      }
    } else {
      cancel_since_ = -1;
      active_ = selected;
    }
    if (stationary_action || command == "CANCEL" || selected == "STOP") {
      result["tool_stop_request"] = true;
    }
    if (selected == "STOP") result["cancel_request"] = true;
    if (command != last_command_) {
      last_command_ = command;
      ++dispatch_id_;
    }
    result["selected_action"] = selected;
    result["action"] = command;
    result["dispatch_id"] = dispatch_id_;
    result["stopped_confirmation"] = stopped;
    return result;
  }

  bool reset() { return policy_.reset(); }

 private:
  Policy policy_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  std::string active_, last_command_;
  double cancel_since_ = -1;
  uint64_t dispatch_id_ = 0;
};
}  // namespace field_navigation
