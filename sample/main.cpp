// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tros_runtime_stats/runtime_stats.h"

class SubNode : public rclcpp::Node {
 public:
  SubNode() : Node("sub_node") {
    sub_ = create_subscription<std_msgs::msg::String>("chatter", 10,
      std::bind(&SubNode::chatter_callback, this, std::placeholders::_1)
    );

    // 延迟构造
    std::thread([this]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      auto param = tros::RuntimeStatsParams(
        this->get_node_base_interface()->get_name(), true);
      param.stats_window_sec = 1.0;
      sp_runtime_stat_ = 
        std::make_shared<tros::RuntimeStats<rclcpp::Node::WeakPtr>>(shared_from_this(), param);
    }).detach();
  }

 private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::shared_ptr<tros::RuntimeStats<rclcpp::Node::WeakPtr>> sp_runtime_stat_;

  void chatter_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    if (!sp_runtime_stat_) return;
    auto msg_ts = this->now();
    sp_runtime_stat_->TrigerOn(msg_ts, this->now());
    int sleep_ms = std::rand() % 100;
    RCLCPP_INFO(this->get_logger(), "sleep %d ms", sleep_ms);
    rclcpp::sleep_for(std::chrono::milliseconds(sleep_ms));
    sp_runtime_stat_->TrigerOff(msg_ts, this->now());
  }
};

int main(int argc, char** arg) {
  rclcpp::init(argc, arg);
  auto node = std::make_shared<SubNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}