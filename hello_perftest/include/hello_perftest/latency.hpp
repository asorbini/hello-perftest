// (c) 2021 Copyright, Real-Time Innovations, Inc.  All rights reserved.
//
// RTI grants Licensee a license to use, modify, compile, and create derivative
// works of the Software.  Licensee has the right to distribute object form
// only for use with RTI products.  The Software is provided "as is", with no
// warranty of any type, including any warranty for fitness for any purpose.
// RTI is under no obligation to maintain or support the Software.  RTI shall
// not be liable for any incidental or consequential damages arising out of the
// use or inability to use the software.
#ifndef HELLO_PERFTEST__LATENCY_PUB_HPP
#define HELLO_PERFTEST__LATENCY_PUB_HPP

#include <chrono>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "hello_perftest_interfaces/msg/test_payload.hpp"

namespace hello_perftest
{
class LatencyNode : public rclcpp::Node
{
public:
  using MessageType = hello_perftest_interfaces::msg::TestPayload;

  LatencyNode(
      const std::string & name,
      const std::string & pub_topic,
      const std::string & sub_topic,
      const rclcpp::NodeOptions & options)
  : Node(name, options)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    pub_ = this->create_publisher<MessageType>(pub_topic, qos);
    sub_ = this->create_subscription<MessageType>(sub_topic, qos,
      [this](const MessageType::SharedPtr payload) {
        on_payload(*payload);
      });
    
    // Create two endpoints that we will use to signal the end of a test.
    pub_exit_ = this->create_publisher<std_msgs::msg::String>("latency_exit", qos);
    sub_exit_ = this->create_subscription<std_msgs::msg::String>("latency_exit", qos,
      [this](const std_msgs::msg::String::SharedPtr exit_msg){
        if (exit_msg->data != this->get_name() && rclcpp::ok()) {
          RCLCPP_INFO_STREAM(this->get_logger(), "exit on message received");
          rclcpp::shutdown();
        }
      });
  }

  virtual void begin() {};

protected:
  virtual void on_payload(const MessageType & payload) = 0;

  rclcpp::Publisher<MessageType>::SharedPtr pub_;
  rclcpp::Subscription<MessageType>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_exit_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_exit_;
  MessageType payload_;
};

class LatencyPublisher : public LatencyNode
{
public:
  LatencyPublisher(const rclcpp::NodeOptions & options)
  : LatencyPublisher(true, options)
  {}

  LatencyPublisher(const bool autostart, const rclcpp::NodeOptions & options,
      std::chrono::nanoseconds autostart_delay = std::chrono::milliseconds(500))
  : LatencyNode("latency_pub", "ping", "pong", options)
  {
    // A latency test should probably be time-driven, but use a max sample
    // count to terminate the test for the sake of simplicity.
    this->declare_parameter("max_samples", 100);
    this->declare_parameter("subscribers_count", 1);

    if (autostart) {
      // start a timer to call begin()
      autostart_timer_ = this->create_wall_timer(autostart_delay,
        [this](){
          begin();
        });
    }
  }

  virtual void begin()
  {
    if (nullptr != autostart_timer_) {
      autostart_timer_->cancel();
      autostart_timer_ = nullptr;
    }

    size_t subscribers_count;
    this->get_parameter("subscribers_count", subscribers_count);
    if (subscribers_count == 0) {
      throw std::runtime_error("subscribers_count must be greater than 0");
    }

    this->get_parameter("max_samples", samples_max_);
    samples_sent_ = 0;
    samples_recv_ = 0;

    this->wait_for_peers(subscribers_count);

    RCLCPP_INFO_STREAM(this->get_logger(), "begin latency test");
    ping();
  }

protected:
  void wait_for_peers(const size_t peers_count)
  {
    using namespace std::chrono_literals;
    rclcpp::Event::SharedPtr graph_event = this->get_graph_event();
    auto peers_discovered = [this, peers_count]() {
      return this->count_subscribers(pub_->get_topic_name()) >= peers_count &&
        this->count_publishers(sub_->get_topic_name()) >= peers_count;
    };
    RCLCPP_INFO_STREAM(this->get_logger(),
      "waiting for " << peers_count << " peers ");
    while (!peers_discovered()) {
      this->wait_for_graph_change(graph_event, 100ms);
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "peers discovered");

    // Note that this function does not guarantee "symmetric discovery",
    // i.e. the remote peers are not guaranteed to have also discovered
    // the publisher. The only way to overcome this issue is to either
    // use "transient local" durability, or to augment the initialization
    // protocol by adding more synchronization between publishers and
    // subscribers. The subscribers could for example make use of a 
    // ROS service exposed by the publisher to notify them of their
    // readiness (the same could be achieved with a standard topic too).
    // To keep things simple, this example creates endpoints with
    // transient local durability.
  }

  void on_complete()
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "latency test complete");
    std_msgs::msg::String exit_msg;
    exit_msg.data = this->get_name();
    pub_exit_->publish(exit_msg);

    autostart_timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
      [this](){
        autostart_timer_->cancel();
        RCLCPP_INFO_STREAM(this->get_logger(), "shutting down publisher");
        rclcpp::shutdown();
      });
  }

  virtual void on_payload(const MessageType & payload)
  {
    (void)payload;
    samples_recv_ += 1;
    RCLCPP_INFO_STREAM(this->get_logger(), "pong received: " << samples_recv_);
    if (samples_sent_ < samples_max_) {
      ping();
    } else {
      on_complete();
    }
  }

  void ping(const bool resend = false)
  {
    // TODO prepare ping payload
    pub_->publish(payload_);
    if (!resend) {
      samples_sent_ += 1;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "ping sent: " << samples_sent_);
  }

  size_t samples_max_{0};
  size_t samples_sent_{0};
  size_t samples_recv_{0};
  rclcpp::TimerBase::SharedPtr autostart_timer_;
};

class LatencySubscriber : public LatencyNode
{
public:
  LatencySubscriber(const rclcpp::NodeOptions & options)
  : LatencySubscriber(true, options)
  {}

  LatencySubscriber(const bool autostart, const rclcpp::NodeOptions & options)
  : LatencyNode("latency_sub", "pong", "ping", options)
  {
    (void)autostart;
    RCLCPP_INFO_STREAM(this->get_logger(), "subscriber ready");
  }

protected:
  virtual void on_payload(const MessageType & payload)
  {
    samples_recv_ += 1;
    payload_.data = payload.data;
    pub_->publish(payload_);
    RCLCPP_INFO_STREAM(this->get_logger(), "pong published " << samples_recv_);
  }

  size_t samples_recv_{0};
};

template<typename T>
int node_main(int argc, char * argv[])
{
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  options.arguments(args);
  
  auto node = std::make_shared<T>(false, options);
  exec.add_node(node);

  node->begin();

  exec.spin();

  rclcpp::shutdown();
  return 0;
}


}  // namespace hello_perftest


#endif  // HELLO_PERFTEST__LATENCY_PUB_HPP
