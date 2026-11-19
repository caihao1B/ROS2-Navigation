#include <cassert>
#include <fstream>
#include <iostream>

#include "field_navigation/policy.hpp"
using field_navigation::Json;
Json input() {
  return {
      {"stale", false},          {"bindings_valid", true}, {"localized", true},
      {"sensors_healthy", true}, {"remaining_s", 600.},    {"stopped", true},
      {"tool_stopped", true},    {"nav_idle", true},       {"blocked", false}};
}
int main(int argc, char** argv) {
  assert(argc == 3);
  Json cfg;
  std::ifstream f(argv[1]);
  f >> cfg;
  int n = 0;
  auto check = [&](Json e, std::string expected, double t = 1.) {
    field_navigation::Policy p(cfg);
    BT::BehaviorTreeFactory b;
    p.register_nodes(b);
    auto tree = b.createTreeFromFile(argv[2]);
    p.prepare(e, t);
    tree.tickOnce();
    assert(p.decision.at("action") == expected);
    ++n;
  };
  auto e = input();
  check(e, "EXECUTE");
  e["unsafe"] = true;
  check(e, "STOP");
  e = input();
  e["stale"] = true;
  check(e, "STOP");
  e = input();
  e["bindings_valid"] = false;
  check(e, "STOP");
  e = input();
  e["localized"] = false;
  check(e, "WAIT");
  e = input();
  e["remaining_s"] = 7.;
  check(e, "STOP");
  e = input();
  e["complete"] = true;
  check(e, "COMPLETE");
  e = input();
  e["blocked"] = true;
  check(e, "WAIT");
  field_navigation::Policy p(cfg);
  BT::BehaviorTreeFactory b;
  p.register_nodes(b);
  auto tree = b.createTreeFromFile(argv[2]);
  auto tick = [&](Json e, double t, std::string expected) {
    p.prepare(e, t);
    tree.tickOnce();
    assert(p.decision.at("action") == expected);
    ++n;
  };
  e = input();
  e["blocked"] = true;
  tick(e, 1., "WAIT");
  tick(e, 6., "WAIT");
  e["detour_valid"] = true;
  e["detour_s"] = 2.;
  tick(e, 7., "DETOUR");
  e["detour_valid"] = false;
  tick(e, 14., "REPLAN");
  e["replans"] = 2;
  e["alternative"] = true;
  tick(e, 16., "DEFER");
  e["alternative"] = false;
  tick(e, 18., "HUMAN");
  e = input();
  tick(e, 19., "HUMAN");
  assert(p.reset());
  tick(e, 20., "EXECUTE");
  e = input();
  e["blocked"] = true;
  e["leader_on_path"] = true;
  e["leader_same_direction"] = true;
  e["leader_confidence"] = .9;
  e["leader_speed"] = .3;
  e["clearance_m"] = 3.;
  e["ttc_s"] = 10.;
  e["path_valid"] = true;
  tick(e, 21., "FOLLOW");
  tick(e, 80., "FOLLOW");
  e["clearance_m"] = 1.;
  tick(e, 81., "WAIT");
  e = input();
  p.prepare(e, 82.);
  assert(p.reset());
  e["commanded_motion"] = true;
  e["progress_m"] = 0.;
  tick(e, 83., "EXECUTE");
  tick(e, 95., "HUMAN");
  e["stopped"] = false;
  p.prepare(e, 96.);
  assert(!p.reset());
  ++n;
  e = input();
  e["patch_active"] = true;
  e["path_valid"] = true;
  check(e, "DETOUR");
  e["stale"] = true;
  check(e, "STOP");
  e = input();
  e["patch_active"] = true;
  e["path_valid"] = false;
  e["blocked"] = true;
  check(e, "WAIT");
  std::cout << Json({{"passed", true},
                     {"cases", n},
                     {"framework", "BehaviorTree.CPP actual XML execution"}})
                   .dump()
            << '\n';
  return 0;
}
