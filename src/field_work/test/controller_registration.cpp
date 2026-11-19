#include <exception>
#include <iostream>
#include <nav2_core/controller.hpp>
#include <pluginlib/class_loader.hpp>

int main() {
  try {
    pluginlib::ClassLoader<nav2_core::Controller> loader(
        "nav2_core", "nav2_core::Controller");
    const auto controller =
        loader.createSharedInstance("field_work::DirectedPathCurvature");
    return controller ? 0 : 1;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
