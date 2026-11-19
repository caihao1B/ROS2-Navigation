#include <fstream>
#include <iostream>
#include <stdexcept>

#include "field_navigation/supervision.hpp"

using field_navigation::Json;

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "usage: inspection_replay POLICY.json TREE.xml < "
                 "OBSERVATIONS.jsonl\n";
    return 2;
  }
  try {
    std::ifstream file(argv[1]);
    auto thresholds = Json::parse(file);
    field_navigation::Supervisor supervisor(thresholds, argv[2]);
    double previous = -1.0;
    std::string line;
    while (std::getline(std::cin, line)) {
      auto input = Json::parse(line);
      const double stamp = input.at("t").get<double>();
      if (!std::isfinite(stamp) || stamp < 0.0 || stamp <= previous) {
        throw std::runtime_error(
            "observations must have increasing simulation timestamps");
      }
      auto decision = supervisor.tick(std::move(input), stamp);
      std::cout << decision.dump() << '\n';
      previous = stamp;
    }
    if (previous < 0)
      throw std::runtime_error("the observation stream was empty");
  } catch (const std::exception &error) {
    std::cerr << "inspection_replay: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
