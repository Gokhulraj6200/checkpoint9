#include <memory>

#include "my_components/attach_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/utilities.hpp"
#include <cstddef>
#include <cstdio>

int main(int argc, char *argv[]) {
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  auto attach_server = std::make_shared<my_components::AttachServer>(options);
  exec.add_node(attach_server);

  // spin will block until work comes in, execute work as it becomes available,
  // and keep blocking. It will only be interrupted by Ctrl-C.
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
