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
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tros_runtime_stats/runtime_stats.h"

namespace tros {
  
struct RuntimeFrameStat {
  builtin_interfaces::msg::Time msg_ts;
  builtin_interfaces::msg::Time msg_recved_ts;
  builtin_interfaces::msg::Time msg_processed_ts;

  RuntimeFrameStat() {}
  RuntimeFrameStat(builtin_interfaces::msg::Time _msg_ts,
      builtin_interfaces::msg::Time _msg_recved_ts) :
      msg_ts(_msg_ts), msg_recved_ts(_msg_recved_ts) {
  }
};

template <typename NodeWeakPtrType>
RuntimeStats<NodeWeakPtrType>::RuntimeStats(const NodeWeakPtrType & parent,
  RuntimeStatsParams param) :
  param_(param) {
  auto node = parent.lock();
  logger_ = node->get_logger();
  parent_node_name_ = node->get_name();
  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(node);

  if (IsEnabled() && param_.publish_stat) {
    RCLCPP_WARN(logger_, "[%s] Publish stat with topic name [%s]",
      param_.node_name.c_str(), topic_name_.c_str());
    pub_stats_ = rclcpp::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(node, topic_name_, 1);
  }
  ParseParams(node);
  Init(node->now());
}

template <typename NodeWeakPtrType>
RuntimeStats<NodeWeakPtrType>::~RuntimeStats() {
}

template <typename NodeWeakPtrType>
void RuntimeStats<NodeWeakPtrType>::AddParamCallback() {
  if (!param_subscriber_) {
    RCLCPP_ERROR(logger_, "param_subscriber_ is null");
    return;
  }

  auto event_cb = [this](const rcl_interfaces::msg::ParameterEvent & parameter_event) {
    if (!parameter_event.changed_parameters.empty()) {
      if (parameter_event.node != "/" + parent_node_name_) {
        RCLCPP_INFO(logger_, "Received parameter event from node \"%s\", this node is \"%s\".",
          parameter_event.node.c_str(),
          parent_node_name_.c_str());
        return;
      }
      RCLCPP_INFO(
        logger_, "Received parameter event from node \"%s\" with %ld parameters changed",
        parameter_event.node.c_str(),
        parameter_event.changed_parameters.size());
    } else {
      return;
    }

    bool has_any_param_parsed = false;
    for (const auto& p : parameter_event.changed_parameters) {
      bool parse_success = true;
      if (GetFullName("enabled") == p.name) {
        param_.enabled = rclcpp::Parameter::from_parameter_msg(p).as_bool();
      } else if (GetFullName("print_stat") == p.name) { 
        param_.print_stat = rclcpp::Parameter::from_parameter_msg(p).as_bool();
      } else if (GetFullName("enable_debug") == p.name) { 
        param_.enable_debug = rclcpp::Parameter::from_parameter_msg(p).as_bool();
      } else if (GetFullName("stats_window_sec") == p.name) { 
        param_.stats_window_sec = rclcpp::Parameter::from_parameter_msg(p).as_double();
      } else if (GetFullName("stats_cache_len_thr") == p.name) { 
        param_.stats_cache_len_thr = rclcpp::Parameter::from_parameter_msg(p).as_int();
      } else if (GetFullName("msg_cache_timeout_thr") == p.name) { 
        param_.msg_cache_timeout_thr = rclcpp::Parameter::from_parameter_msg(p).as_double();
      } else if (GetFullName("msg_cache_len_thr") == p.name) { 
        param_.msg_cache_len_thr = rclcpp::Parameter::from_parameter_msg(p).as_int();
      } else if (GetFullName("proc_delay_warn_thr") == p.name) { 
        param_.proc_delay_warn_thr = rclcpp::Parameter::from_parameter_msg(p).as_double();
      } else if (GetFullName("warn2file") == p.name) { 
        param_.warn2file = rclcpp::Parameter::from_parameter_msg(p).as_bool();
      } else if (GetFullName("file_path") == p.name) { 
        param_.file_path = rclcpp::Parameter::from_parameter_msg(p).as_string();
      } else {
        parse_success = false;
        RCLCPP_INFO(
          logger_, "Inside event: \"%s\" is not supported in '%s'",
          p.name.c_str(),
          (GetPrefixName()).c_str()
        );
      }
      if (parse_success) {
        if (!has_any_param_parsed) {
          has_any_param_parsed = true;
        }
        RCLCPP_WARN(
          logger_, "Inside event: \"%s\" changed to %s",
          p.name.c_str(),
          rclcpp::Parameter::from_parameter_msg(p).value_to_string().c_str());
      }
    }
    if (has_any_param_parsed) {
      PrintParam();
    }
  };
  event_cb_handle_ = param_subscriber_->add_parameter_event_callback(event_cb);
}

template <typename NodeWeakPtrType>
void RuntimeStats<NodeWeakPtrType>::Init(builtin_interfaces::msg::Time stamp) {
  msg_cache_.clear();
  while (!stats_cache_.empty()) {
    stats_cache_.pop();
  }
    
  AddParamCallback();

  RCLCPP_WARN(logger_,
    "Runtime stats in parent_node [%s] node [%s] ns [%s] is %s",
    parent_node_name_.c_str(),
    param_.node_name.c_str(),
    param_.ns_name.c_str(),
    (IsEnabled()? "enabled" : "disabled")
  );
  if (IsEnabled()) {
    PrintParam();
    if (param_.warn2file && param_.proc_delay_warn_thr > 0.0f) {
      std::string fname = param_.file_path + "/" +
        parent_node_name_ + "_";
      if (!param_.ns_name.empty()) {
        fname += param_.ns_name + "_";
      }
      fname = fname +
        param_.node_name + "_";
      fname += std::to_string(stamp.sec) + ".log";
      ofs_log_.open(fname, std::ios::out);
      if (ofs_log_ && ofs_log_.is_open()) {
        RCLCPP_WARN(logger_,
          "[%s] create file [%s] success",
          param_.node_name.c_str(),
          fname.c_str()
        );
      } else {
        RCLCPP_ERROR(logger_,
          "[%s] cannot create file in path [%s]",
          param_.node_name.c_str(),
          param_.file_path.c_str()
        );
      }
    }
  }
}

template <typename NodeWeakPtrType>
void RuntimeStats<NodeWeakPtrType>::PrintParam() {
  RCLCPP_WARN(logger_,
    "\n               ns_name: %s" \
    "\n             node_name: %s" \
    "\n               enabled: %s" \
    "\n            print_stat: %s" \
    "\n          publish_stat: %s"
    "\n          enable_debug: %s (warn log level)" \
    "\n      stats_window_sec: %.2f" \
    "\n   stats_cache_len_thr: %d" \
    "\n msg_cache_timeout_thr: %.2f" \
    "\n     msg_cache_len_thr: %d"
    "\n   proc_delay_warn_thr: %.3f" \
    "\n             warn2file: %s" \
    "\n             file_path: %s",
    param_.ns_name.c_str(),
    param_.node_name.c_str(),
    (param_.enabled? "true" : "false"),
    (param_.print_stat? "true" : "false"),
    (param_.publish_stat? (std::string("true (topic name `" + topic_name_ + "`)").data()) : "false"),
    (param_.enable_debug? "true" : "false"),
    param_.stats_window_sec,
    param_.stats_cache_len_thr,
    param_.msg_cache_timeout_thr,
    param_.msg_cache_len_thr,
    param_.proc_delay_warn_thr,
    (param_.warn2file? "true" : "false"),
    param_.file_path.c_str()
  );
}

template <typename NodeWeakPtrType>
bool RuntimeStats<NodeWeakPtrType>::IsEnabled() {
  return param_.enabled;
}

template <typename NodeWeakPtrType>
RuntimeStatsErrCode RuntimeStats<NodeWeakPtrType>::TrigerOn(const builtin_interfaces::msg::Time& msg_ts,
  const builtin_interfaces::msg::Time& now_ts) {
  if (!IsEnabled()) {
    RCLCPP_WARN_ONCE(logger_,
      "[%s] Runtime stats is disabled, this msg appears only once.", 
      param_.node_name.c_str());
    return RuntimeStatsErrCode::DISABLED;
  }
  // Check with ts
  auto time_diff = (rclcpp::Time(now_ts) - rclcpp::Time(msg_ts)).seconds();
  if (time_diff < 0) {
    RCLCPP_ERROR(logger_,
      "[%s] TrigerOn Check time failed! ts in msg (%d.%d) is later than now (%d.%d), time diff: %.2f",
      param_.node_name.c_str(),
      msg_ts.sec, msg_ts.nanosec, now_ts.sec, now_ts.nanosec, time_diff
    );
    return RuntimeStatsErrCode::INVALID_STAMP;
  }

  auto lk = std::lock_guard(stat_mtx_);
  msg_cache_[rclcpp::Time(msg_ts).seconds()] =
    RuntimeFrameStat(msg_ts, now_ts);

  if (param_.enable_debug) {
    RCLCPP_WARN(logger_,
      "[%s] TrigerOn with ts (%d.%d), cache size: %ld",
      param_.node_name.c_str(),
      msg_ts.sec, msg_ts.nanosec, msg_cache_.size());
  }
  
  // Del msg if cache size > cache_len_thr
  while (rclcpp::ok() &&
    static_cast<int>(msg_cache_.size()) > param_.msg_cache_len_thr) {
    auto begin = msg_cache_.begin();
    RCLCPP_WARN(logger_,
      "[%s] msg cache exceeds limit (%d), del msg with ts (%d.%d)",
      param_.node_name.c_str(),
      param_.msg_cache_len_thr,
      msg_ts.sec, msg_ts.nanosec);
    msg_cache_.erase(begin);
  }
  // Del msg if timeout
  while (rclcpp::ok() && !msg_cache_.empty()) {
    auto begin = msg_cache_.begin();
    float time_diff = (rclcpp::Time(now_ts) - rclcpp::Time(begin->second.msg_ts)).seconds();
    if (time_diff > param_.msg_cache_timeout_thr) {
      RCLCPP_WARN(logger_,
        "[%s] msg ts diff (%.2f sec) exceeds limit (%.2f sec), del msg:" \
        "\n msg ts (%d.%d)" \
        "\n now ts (%d.%d)",
        param_.node_name.c_str(),
        time_diff,
        param_.msg_cache_timeout_thr,
        begin->second.msg_ts.sec, begin->second.msg_ts.nanosec,
        now_ts.sec, now_ts.nanosec
      );
      msg_cache_.erase(begin);
    } else {
      break;
    }
  }

  return RuntimeStatsErrCode::SUCCESS;
}

template <typename NodeWeakPtrType>
RuntimeStatsErrCode RuntimeStats<NodeWeakPtrType>::TrigerOff(const builtin_interfaces::msg::Time& msg_ts,
  const builtin_interfaces::msg::Time& now_ts,
  std::shared_ptr<RuntimeStatsOutput>& output) {
  if (!IsEnabled()) {
    RCLCPP_WARN_ONCE(logger_,
      "[%s] Runtime stats is disabled, this msg appears only once.", 
      param_.node_name.c_str());
    return RuntimeStatsErrCode::DISABLED;
  }
  output = nullptr;
  auto time_diff = (rclcpp::Time(now_ts) - rclcpp::Time(msg_ts)).seconds();
  if (time_diff < 0) {
    RCLCPP_ERROR(logger_,
      "[%s] TrigerOff Check time failed! ts in msg (%d.%d) is later than now (%d.%d), time diff: %.2f",
      param_.node_name.c_str(),
      msg_ts.sec, msg_ts.nanosec, now_ts.sec, now_ts.nanosec, time_diff
    );
    return RuntimeStatsErrCode::INVALID_STAMP;
  }
  
  auto lk = std::lock_guard(stat_mtx_);

  if (param_.enable_debug) {
    RCLCPP_WARN(logger_,
      "[%s] TrigerOff with ts (%d.%d), cache size: %ld",
      param_.node_name.c_str(),
      msg_ts.sec, msg_ts.nanosec, msg_cache_.size());
  }

  auto val = msg_cache_.find(rclcpp::Time(msg_ts).seconds());
  if (val == msg_cache_.end()) {
    RCLCPP_ERROR(logger_,
      "[%s] TrigerOff Find ts (%d.%d) failed in msg cache",
      param_.node_name.c_str(),
      msg_ts.sec, msg_ts.nanosec);
    return RuntimeStatsErrCode::INVALID_STAMP;
  }
  val->second.msg_processed_ts = now_ts;
  stats_cache_.push(val->second);
  msg_cache_.erase(val);
  
  if ((rclcpp::Time(now_ts) - rclcpp::Time(stats_cache_.front().msg_ts)).seconds() >=
      param_.stats_window_sec) {
    output = std::make_shared<RuntimeStatsOutput>();
    size_t frame_num = stats_cache_.size();
    float time_diff =
      (rclcpp::Time(stats_cache_.back().msg_ts) - rclcpp::Time(stats_cache_.front().msg_ts)).seconds();
    if (time_diff <= 0 || frame_num == 0) {
      RCLCPP_WARN(logger_,
        "[%s] Invalid time diff: %.2f, frame_num: %ld",
        param_.node_name.c_str(),
        time_diff, frame_num);
      while (!stats_cache_.empty() && rclcpp::ok()) {
        stats_cache_.pop();
      }
      return RuntimeStatsErrCode::INVALID_STAMP;
    }
    output->output_fps = frame_num / time_diff;

    double input_delay_sum = 0;
    double process_delay_sum = 0;
    double output_delay_sum = 0;
    while (!stats_cache_.empty()) {
      auto frame_stat = stats_cache_.front();
      stats_cache_.pop();
      float input_delay =
        (rclcpp::Time(frame_stat.msg_recved_ts) - rclcpp::Time(frame_stat.msg_ts)).seconds();
      float process_delay =
        (rclcpp::Time(frame_stat.msg_processed_ts) - rclcpp::Time(frame_stat.msg_recved_ts)).seconds();
      float output_delay =
        (rclcpp::Time(frame_stat.msg_processed_ts) - rclcpp::Time(frame_stat.msg_ts)).seconds();

      output->input_delay_min = std::min(output->input_delay_min, input_delay);
      output->input_delay_max = std::max(output->input_delay_max, input_delay);
      input_delay_sum += input_delay;

      output->process_delay_min = std::min(output->process_delay_min, process_delay);
      output->process_delay_max = std::max(output->process_delay_max, process_delay);
      process_delay_sum += process_delay;

      output->output_delay_min = std::min(output->output_delay_min, output_delay);
      output->output_delay_max = std::max(output->output_delay_max, output_delay);
      output_delay_sum += output_delay;
    }

    output->input_delay_avg = input_delay_sum / static_cast<double>(frame_num);
    output->process_delay_avg = process_delay_sum / static_cast<double>(frame_num);
    output->output_delay_avg = output_delay_sum / static_cast<double>(frame_num);

    if (param_.print_stat) {
      RCLCPP_WARN(logger_,
        "perf in window [%.2f] sec: " \
        "\n      ns: %s" \
        "\n    node: %s" \
        "\n out fps: %.2f" \
        "\n   delay: min   | max   | avg" \
        "\n   input: %.3f | %.3f | %.3f" \
        "\n    proc: %.3f | %.3f | %.3f" \
        "\n  output: %.3f | %.3f | %.3f",
        param_.stats_window_sec,
        param_.ns_name.c_str(),
        param_.node_name.c_str(),
        output->output_fps,
        output->input_delay_min,
        output->input_delay_max,
        output->input_delay_avg,
        output->process_delay_min,
        output->process_delay_max,
        output->process_delay_avg,
        output->output_delay_min,
        output->output_delay_max,
        output->output_delay_avg
      );
    }
    if (param_.publish_stat && pub_stats_ && pub_stats_->get_subscription_count() > 0) {
      auto msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
      msg->header.stamp = msg_ts;
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.name = "tros_perf";
      status.message = param_.ns_name + "." + param_.node_name + ", delay(min max avg)";
      {
        diagnostic_msgs::msg::KeyValue value;
        value.key = "input";
        std::stringstream ss;
        ss << std::setprecision(3) << std::fixed << output->input_delay_min << " | " << output->input_delay_max << " | " << output->input_delay_avg;
        value.value = ss.str();
        status.values.push_back(value);
      }
      {
        diagnostic_msgs::msg::KeyValue value;
        value.key = "proc";
        std::stringstream ss;
        ss << std::setprecision(3) << std::fixed << output->process_delay_min << " | " << output->process_delay_max << " | " << output->process_delay_avg;
        value.value = ss.str();
        status.values.push_back(value);
      }
      {
        diagnostic_msgs::msg::KeyValue value;
        value.key = "output";
        std::stringstream ss;
        ss << std::setprecision(3) << std::fixed << output->output_delay_min << " | " << output->output_delay_max << " | " << output->output_delay_avg;
        value.value = ss.str();
        status.values.push_back(value);
      }

      msg->status.push_back(status);
      pub_stats_->publish(std::move(msg));
    }

    if (param_.proc_delay_warn_thr > 0 && output->process_delay_max > param_.proc_delay_warn_thr) {
      RCLCPP_WARN(logger_,
        "process_delay_max [%.3f] exceeds thr [%.3f] in node [%s]",
        output->process_delay_max,
        param_.proc_delay_warn_thr,
        param_.node_name.c_str()
      );

      if (param_.warn2file) {
        if (ofs_log_ && ofs_log_.is_open()) {
          ofs_log_ << msg_ts.sec << "." << msg_ts.nanosec
            << std::setprecision(3) << std::fixed
            << "\t input_delay_max: " << output->input_delay_max
            << "\t process_delay_max: " << output->process_delay_max
            << "\t output_delay_max: " << output->output_delay_max
            << "\n";
        } else {
          RCLCPP_ERROR_ONCE(logger_,
            "[%s] file is not openned in path [%s], write proc delay warn log to file failed, this log appears only once",
            param_.node_name.c_str(),
            param_.file_path.c_str()
          );
        }
      }
    } 
  }

  while (rclcpp::ok() &&
    static_cast<int>(stats_cache_.size()) > param_.stats_cache_len_thr) {
    stats_cache_.pop();
  }
  
  return RuntimeStatsErrCode::SUCCESS;
}

template <typename NodeWeakPtrType>
RuntimeStatsErrCode RuntimeStats<NodeWeakPtrType>::TrigerOff(const builtin_interfaces::msg::Time& msg_ts,
  const builtin_interfaces::msg::Time& now_ts) {
  if (!IsEnabled()) {
    RCLCPP_WARN_ONCE(logger_,
      "[%s] Runtime stats is not enabled, this msg appears only once.",
      param_.node_name.c_str());
    return RuntimeStatsErrCode::DISABLED;
  }
  std::shared_ptr<RuntimeStatsOutput> output;
  return TrigerOff(msg_ts, now_ts, output);
}

template <typename NodeWeakPtrType>
RuntimeStatsErrCode RuntimeStats<NodeWeakPtrType>::EraseTs(const builtin_interfaces::msg::Time& msg_ts) {
  if (!IsEnabled()) {
    RCLCPP_WARN_ONCE(logger_,
      "[%s] Runtime stats is not enabled, this msg appears only once.",
      param_.node_name.c_str());
    return RuntimeStatsErrCode::DISABLED;
  }
  
  auto lk = std::lock_guard(stat_mtx_);
  if (param_.enable_debug) {
    RCLCPP_WARN(logger_,
      "[%s] EraseTs with ts (%d.%d), cache size: %ld",
      param_.node_name.c_str(),
      msg_ts.sec, msg_ts.nanosec, msg_cache_.size());
  }
  auto val = msg_cache_.find(rclcpp::Time(msg_ts).seconds());
  if (val == msg_cache_.end()) {
    // RCLCPP_ERROR(logger_,
    //   "[%s] EraseTs Find ts (%d.%d) failed in msg cache",
    //   param_.node_name.c_str(),
    //   msg_ts.sec, msg_ts.nanosec);
    return RuntimeStatsErrCode::INVALID_STAMP;
  }
  msg_cache_.erase(val);

  return RuntimeStatsErrCode::SUCCESS;
}

template <typename NodeWeakPtrType>
std::string RuntimeStats<NodeWeakPtrType>::GetPrefixName() {
  std::string prefix_name = "";
  if (!param_.ns_name.empty()) {
    prefix_name += param_.ns_name + ".";
  }
  if (!param_.node_name.empty()) {
    prefix_name += param_.node_name + ".";
  }
  prefix_name += module_name_ + ".";
  
  RCLCPP_WARN_ONCE(logger_,
    "[%s] prefix_name: '%s', this msg appears only once.",
    param_.node_name.c_str(),
    prefix_name.c_str());
  return prefix_name;
}

template <typename NodeWeakPtrType>
std::string RuntimeStats<NodeWeakPtrType>::GetFullName(std::string param_name) {
  return GetPrefixName() + param_name;
}

template class RuntimeStats<rclcpp::Node::WeakPtr>;
template class RuntimeStats<rclcpp_lifecycle::LifecycleNode::WeakPtr>;

}