#pragma once
#include <behaviortree_cpp/bt_factory.h>

#include <algorithm>
#include <cmath>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>
namespace field_navigation {
using Json = nlohmann::json;
class Policy {
 public:
  Json thresholds, observation = Json::object(), decision = Json::object();
  double blocked_since = -1, clear_since = -1, progress_since = -1,
         last_progress = 0;
  bool manual = false;
  std::string manual_reason;
  explicit Policy(Json config) : thresholds(std::move(config)) {}
  double number(const char* key, double fallback = 0) const {
    return observation.contains(key) && observation[key].is_number() &&
                   std::isfinite(observation[key].get<double>())
               ? observation[key].get<double>()
               : fallback;
  }
  bool flag(const char* key, bool fallback = false) const {
    return observation.contains(key) && observation[key].is_boolean()
               ? observation[key].get<bool>()
               : fallback;
  }
  double duration() const {
    return blocked_since < 0 ? 0 : number("t") - blocked_since;
  }
  bool fresh() const { return !flag("stale", true); }
  void prepare(Json context, double time) {
    observation = std::move(context);
    observation["t"] = time;
    if (fresh()) {
      if (flag("blocked")) {
        if (blocked_since < 0) blocked_since = time;
        clear_since = -1;
      } else {
        if (clear_since < 0) clear_since = time;
        if (time - clear_since >= thresholds.at("clear_hold_s").get<double>())
          blocked_since = -1;
      }
      if (leader())
        blocked_since =
            time;  // A moving lead vehicle resets the obstruction timer.
      if (flag("commanded_motion") && flag("localized") &&
          (!flag("blocked") || leader())) {
        if (progress_since < 0 ||
            number("progress_m") - last_progress >=
                thresholds.at("progress_delta_m").get<double>()) {
          progress_since = time;
          last_progress = number("progress_m");
        }
        if (time - progress_since >=
            thresholds.at("progress_timeout_s").get<double>()) {
          manual = true;
          manual_reason = "MOTION_WITHOUT_OBSERVED_PROGRESS";
        }
      } else
        progress_since = -1;
    }
    decision = Json::object();
  }
  bool emergency() const {
    return flag("unsafe") || number("risk", 0) >= 1. || flag("collision");
  }
  bool version_bad() const { return !flag("bindings_valid"); }
  bool localized_bad() const {
    return !flag("localized") || !flag("sensors_healthy");
  }
  bool budget_bad() const {
    return number("remaining_s") <=
           thresholds.at("stop_reserve_s").get<double>() + number("next_min_s");
  }
  bool blocked() const { return flag("blocked") || blocked_since >= 0; }
  bool leader() const {
    return flag("blocked") && flag("leader_on_path") &&
           flag("leader_same_direction") &&
           number("leader_confidence") >= .75 && number("leader_speed") > .05 &&
           number("clearance_m") >=
               thresholds.at("follow_min_gap_m").get<double>() &&
           number("ttc_s", 0) >=
               thresholds.at("follow_min_ttc_s").get<double>() &&
           flag("path_valid");
  }
  bool short_wait() const {
    return duration() < thresholds.at("short_wait_s").get<double>() ||
           (!flag("blocked") && blocked_since >= 0);
  }
  bool detour() const {
    return flag("detour_valid") &&
           number("detour_s", 1e6) +
                   thresholds.at("stop_reserve_s").get<double>() <
               number("remaining_s") &&
           number("detour_s", 1e6) <
               thresholds.at("persistent_block_s").get<double>() - duration();
  }
  bool replan() const {
    return duration() >= thresholds.at("persistent_block_s").get<double>() &&
           number("replans") < thresholds.at("max_replans").get<int>() &&
           number("remaining_s") >
               number("replan_s", 10) +
                   thresholds.at("stop_reserve_s").get<double>();
  }
  bool defer() const {
    return duration() >= thresholds.at("persistent_block_s").get<double>() &&
           flag("alternative") &&
           number("deferrals") < thresholds.at("max_deferrals").get<int>();
  }
  bool exhausted() const {
    if (leader()) return false;
    return duration() >= thresholds.at("max_block_s").get<double>() ||
           (duration() >= thresholds.at("persistent_block_s").get<double>() &&
            !replan() && !defer());
  }
  void choose(const std::string& action, const std::string& reason) {
    double cap = thresholds.at("speed_cap_mps");
    if (action == "FOLLOW")
      cap = std::max(
          0.,
          std::min(
              cap,
              number("leader_speed") +
                  .35 * (number("clearance_m") -
                         thresholds.at("follow_target_gap_m").get<double>())));
    if (action == "HUMAN") {
      manual = true;
      manual_reason = reason;
    }
    Json alternatives = Json::array(
        {{{"action", "WAIT"},
          {"eligible", !exhausted()},
          {"reason", "bounded observation wait with clear hysteresis"}},
         {{"action", "FOLLOW"},
          {"eligible", leader()},
          {"reason",
           "same-direction tracked leader, clearance and TTC bounds"}},
         {{"action", "DETOUR"},
          {"eligible", detour()},
          {"reason",
           "only a validated live footprint path with sufficient budget"}},
         {{"action", "REPLAN"},
          {"eligible", replan()},
          {"reason", "persistent blockage and remaining planner attempts"}},
         {{"action", "DEFER"},
          {"eligible", defer()},
          {"reason",
           "another obligation exists; retain residual and original "
           "denominator"}}});
    for (auto& c : alternatives) c["selected"] = c["action"] == action;
    decision = {{"t", number("t")},
                {"action", action},
                {"reason_code", reason},
                {"speed_cap_mps", cap},
                {"blocked_duration_s", duration()},
                {"manual_latched", manual},
                {"evidence", observation},
                {"thresholds", thresholds},
                {"alternatives", alternatives},
                {"infeasibility_claim",
                 "No global no-path proof; bounded search results only"},
                {"work_credit", false}};
  }
  bool reset() {
    if (!flag("stopped") || !flag("tool_stopped") || !flag("nav_idle") ||
        !fresh() || emergency() || version_bad() || localized_bad())
      return false;
    manual = false;
    manual_reason.clear();
    blocked_since = -1;
    clear_since = -1;
    progress_since = -1;
    return true;
  }
  void register_nodes(BT::BehaviorTreeFactory& factory) {
    auto condition = [&](const std::string& name, auto predicate) {
      factory.registerSimpleCondition(name, [this, predicate](BT::TreeNode&) {
        return (this->*predicate)() ? BT::NodeStatus::SUCCESS
                                    : BT::NodeStatus::FAILURE;
      });
    };
    condition("Emergency", &Policy::emergency);
    condition("WrongVersion", &Policy::version_bad);
    condition("LocalizationMissing", &Policy::localized_bad);
    condition("BudgetLow", &Policy::budget_bad);
    condition("Blocked", &Policy::blocked);
    condition("LeaderSuitable", &Policy::leader);
    condition("ShortWait", &Policy::short_wait);
    condition("ValidDetour", &Policy::detour);
    condition("CanReplan", &Policy::replan);
    condition("CanDefer", &Policy::defer);
    condition("BlockExhausted", &Policy::exhausted);
    factory.registerSimpleCondition("InputStale", [this](BT::TreeNode&) {
      return fresh() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    });
    factory.registerSimpleCondition("ManualHold", [this](BT::TreeNode&) {
      return manual ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
    factory.registerSimpleCondition("CompleteFeedback", [this](BT::TreeNode&) {
      return flag("complete") ? BT::NodeStatus::SUCCESS
                              : BT::NodeStatus::FAILURE;
    });
    factory.registerSimpleCondition(
        "ActiveObservedPatch", [this](BT::TreeNode&) {
          return flag("patch_active") && flag("path_valid")
                     ? BT::NodeStatus::SUCCESS
                     : BT::NodeStatus::FAILURE;
        });
    auto action = [&](std::string node, std::string command,
                      std::string reason) {
      factory.registerSimpleAction(
          node, [this, command, reason](BT::TreeNode&) {
            choose(command, reason == "LATCHED" ? manual_reason : reason);
            return BT::NodeStatus::SUCCESS;
          });
    };
    action("ExecuteObservedPatch", "DETOUR",
           "REPLANNED_PATH_CURRENT_PREFIX_VALID");
    action("EmergencyStop", "STOP", "SAFETY_STOP");
    action("VersionStop", "STOP", "VERSION_MISMATCH");
    action("StaleStop", "STOP", "INPUT_STALE");
    action("LocalizationWait", "WAIT", "LOCALIZATION_ADMISSION_WAIT");
    action("BudgetStop", "STOP", "BUDGET_RESERVE_STOP");
    action("ReportLatched", "HUMAN", "LATCHED");
    action("ReportBlocked", "HUMAN", "BOUNDED_REPLAN_EXHAUSTED");
    action("WaitClear", "WAIT", "SHORT_OBSTRUCTION_OR_CLEAR_HYSTERESIS");
    action("WaitBounded", "WAIT", "PERSISTENCE_THRESHOLD_NOT_REACHED");
    action("FollowLeader", "FOLLOW", "SAME_DIRECTION_CLEARANCE_CONTROL");
    action("UseDetour", "DETOUR", "VALIDATED_DETOUR_COST_BENEFIT");
    action("Replan", "REPLAN", "PERSISTENT_BLOCK_GLOBAL_QUERY");
    action("Defer", "DEFER", "TRY_OTHER_OBLIGATION_RETAIN_RESIDUAL");
    action("Complete", "COMPLETE", "FEEDBACK_CONFIRMED_COMPLETE");
    action("Execute", "EXECUTE", "NEXT_OBLIGATION");
  }
};
}  // namespace field_navigation
