// Copyright 2021 Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.

#include "hello_perftest/latency.hpp"

int main(int argc, char * argv[])
{
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  options.arguments(args);
  
  auto publisher = std::make_shared<hello_perftest::LatencyPublisher>(false, options);
  auto subscriber = std::make_shared<hello_perftest::LatencySubscriber>(false, options);
  
  exec.add_node(publisher);
  exec.add_node(subscriber);

  publisher->begin();

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
