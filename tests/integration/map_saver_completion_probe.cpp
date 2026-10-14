#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <slam_toolbox/map_saver.hpp>

std::string exporter_command;

extern "C" int system(const char *command) noexcept {
  exporter_command = command;
  return 0;
}

class MapSaverProbe : public map_saver::MapSaver {
public:
  using map_saver::MapSaver::MapSaver;
  using map_saver::MapSaver::saveMapCallback;
};

void require(bool condition, const char *message) {
  if (!condition)
    throw std::runtime_error(message);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<rclcpp::Node>("map_saver_completion_probe");
    MapSaverProbe saver(node, "/probe_map");
    auto request = std::make_shared<slam_toolbox::srv::SaveMap::Request>();
    auto response = std::make_shared<slam_toolbox::srv::SaveMap::Response>();
    require(!saver.saveMapCallback(nullptr, request, response), "missing map was accepted");
    require(exporter_command.empty(), "exporter ran without a map");
    auto publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/probe_map", 1);
    nav_msgs::msg::OccupancyGrid map;
    map.info.width = map.info.height = 1;
    map.info.resolution = 0.01;
    map.data = {0};
    for (int i = 0; i < 20; ++i) {
      publisher->publish(map);
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    request->name.data = "room'name";
    auto began = std::chrono::steady_clock::now();
    require(saver.saveMapCallback(nullptr, request, response), "map export did not complete");
    const auto milliseconds =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count();
    require(response->result == response->RESULT_SUCCESS, "successful exporter response lost");
    require(milliseconds < 500, "completed exporter still imposes a fixed wait");
    require(exporter_command.find("'room'\\''name'") != std::string::npos,
            "filename quoting changed");
    require(exporter_command.find("--fmt pgm --mode trinary --free 0.196 --occ 0.65") !=
                std::string::npos,
            "raster contract changed");
    request->name.data.clear();
    require(saver.saveMapCallback(nullptr, request, response),
            "default-name export did not complete");
    require(exporter_command.find(" -f ") == std::string::npos, "default-name branch changed");
    std::cout << "fresh_map_saver=passed; missing_map=passed; raster_contract=passed; "
                 "filename_quoting=passed; completion_ms="
              << milliseconds << '\n';
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
