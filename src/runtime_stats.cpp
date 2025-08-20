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

RuntimeStats::RuntimeStats(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  RuntimeStatsParams param) :
  param_(param) {
  auto node = parent.lock();
  logger_ = node->get_logger();
  
  ParseParams<rclcpp_lifecycle::LifecycleNode::SharedPtr>(node);
  Init(node->now());
}

RuntimeStats::RuntimeStats(const rclcpp::Node::WeakPtr & parent,
  RuntimeStatsParams param) :
  param_(param) {
  auto node = parent.lock();
  logger_ = node->get_logger();
  
  ParseParams<rclcpp::Node::SharedPtr>(node);
  Init(node->now());
}

RuntimeStats::~RuntimeStats() {
}

void RuntimeStats::Init(builtin_interfaces::msg::Time stamp) {
  msg_cache_.clear();
  while (!stats_cache_.empty()) {
    stats_cache_.pop();
  }
  
  RCLCPP_WARN(logger_,
    "Runtime stats in module [%s] is %s",
    param_.module_name.c_str(),
    (IsEnabled()? "enabled" : "disabled")
  );
  if (IsEnabled()) {
    RCLCPP_WARN(logger_,
      "\n           module_name: %s" \
      "\n               nm_name: %s" \
      "\n               enabled: %s" \
      "\n            print_stat: %s" \
      "\n          enable_debug: %s" \
      "\n      stats_window_sec: %.2f" \
      "\n   stats_cache_len_thr: %d" \
      "\n msg_cache_timeout_thr: %.2f" \
      "\n     msg_cache_len_thr: %d"
      "\n   proc_delay_warn_thr: %.3f" \
      "\n             warn2file: %s" \
      "\n             file_path: %s",
      param_.module_name.c_str(),
      param_.nm_name.c_str(),
      (param_.enabled? "true" : "false"),
      (param_.print_stat? "true" : "false"),
      (param_.enable_debug? "true" : "false"),
      param_.stats_window_sec,
      param_.stats_cache_len_thr,
      param_.msg_cache_timeout_thr,
      param_.msg_cache_len_thr,
      param_.proc_delay_warn_thr,
      (param_.warn2file? "true" : "false"),
      param_.file_path.c_str()
    );

    if (param_.warn2file && param_.proc_delay_warn_thr > 0.0f) {
      std::string fname = param_.file_path + "/" +
        param_.module_name + "_" +
        param_.nm_name + "_" +
        std::to_string(stamp.sec) +
        ".log";
      ofs_log_.open(fname, std::ios::out);
      if (ofs_log_ && ofs_log_.is_open()) {
        RCLCPP_WARN(logger_,
          "[%s] create file [%s] success",
          param_.module_name.c_str(),
          fname.c_str()
        );
      } else {
        RCLCPP_ERROR(logger_,
          "[%s] cannot create file in path [%s]",
          param_.module_name.c_str(),
          param_.file_path.c_str()
        );
      }
    }
  }
}

bool RuntimeStats::IsEnabled() {
  return param_.enabled;
}

RuntimeStatsErrCode RuntimeStats::TrigerOn(const builtin_interfaces::msg::Time& msg_ts,
  const builtin_interfaces::msg::Time& now_ts) {
  if (!IsEnabled()) {
    RCLCPP_WARN_ONCE(logger_,
      "[%s] Runtime stats is disabled, this msg appears only once.", 
      param_.module_name.c_str());
    return RuntimeStatsErrCode::DISABLED;
  }
  // Check with ts
  auto time_diff = (rclcpp::Time(now_ts) - rclcpp::Time(msg_ts)).seconds();
  if (time_diff < 0) {
    RCLCPP_ERROR(logger_,
      "[%s] Check time failed! ts in msg (%d.%d) is later than now (%d.%d), time diff: %.2f",
      param_.module_name.c_str(),
      msg_ts.sec, msg_ts.nanosec, now_ts.sec, now_ts.nanosec, time_diff
    );
    return RuntimeStatsErrCode::INVALID_STAMP;
  }

  auto lk = std::lock_guard(stat_mtx_);
  msg_cache_[rclcpp::Time(msg_ts).seconds()] =
    RuntimeFrameStat(msg_ts, now_ts);

  if (param_.enable_debug) {
    RCLCPP_INFO(logger_,
      "[%s] TrigerOn with ts (%d.%d), cache size: %ld",
      param_.module_name.c_str(),
      msg_ts.sec, msg_ts.nanosec, msg_cache_.size());
  }
  
  // Del msg if cache size > cache_len_thr
  while (rclcpp::ok() &&
    static_cast<int>(msg_cache_.size()) > param_.msg_cache_len_thr) {
    auto begin = msg_cache_.begin();
    RCLCPP_WARN(logger_,
      "[%s] msg cache exceeds limit (%d), del msg with ts (%d.%d)",
      param_.module_name.c_str(),
      param_.msg_cache_len_thr,
      msg_ts.sec, msg_ts.nanosec);
    msg_cache_.erase(begin);
  }
  // Del msg if timeout
  while (rclcpp::ok()) {
    auto begin = msg_cache_.begin();
    float time_diff = (rclcpp::Time(now_ts) - rclcpp::Time(begin->second.msg_ts)).seconds();
    if (time_diff > param_.msg_cache_timeout_thr) {
      RCLCPP_WARN(logger_,
        "[%s] msg ts diff (%.2f sec) exceeds limit (%.2f sec), del msg! " \
        "\n msg ts (%d.%d)" \
        "\n now ts (%d.%d)",
        param_.module_name.c_str(),
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

RuntimeStatsErrCode RuntimeStats::TrigerOff(const builtin_interfaces::msg::Time& msg_ts,
  const builtin_interfaces::msg::Time& now_ts,
  std::shared_ptr<RuntimeStatsOutput>& output) {
  if (!IsEnabled()) {
    RCLCPP_WARN_ONCE(logger_, "Runtime stats is not enabled, this msg appears only once.");
    return RuntimeStatsErrCode::DISABLED;
  }
  output = nullptr;
  auto time_diff = (rclcpp::Time(now_ts) - rclcpp::Time(msg_ts)).seconds();
  if (time_diff < 0) {
    RCLCPP_ERROR(logger_,
      "[%s] Check time failed! ts in msg (%d.%d) is later than now (%d.%d), time diff: %.2f",
      param_.module_name.c_str(),
      msg_ts.sec, msg_ts.nanosec, now_ts.sec, now_ts.nanosec, time_diff
    );
    return RuntimeStatsErrCode::INVALID_STAMP;
  }
  
  auto lk = std::lock_guard(stat_mtx_);

  if (param_.enable_debug) {
    RCLCPP_INFO(logger_,
      "[%s] TrigerOff with ts (%d.%d), cache size: %ld",
      param_.module_name.c_str(),
      msg_ts.sec, msg_ts.nanosec, msg_cache_.size());
  }

  auto val = msg_cache_.find(rclcpp::Time(msg_ts).seconds());
  if (val == msg_cache_.end()) {
    RCLCPP_ERROR(logger_,
      "[%s] Find ts (%d.%d) failed in msg cache",
      param_.module_name.c_str(),
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
        param_.module_name.c_str(),
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
        "\n  module: %s" \
        "\n out fps: %.2f" \
        "\n   delay: min   | max   | avg" \
        "\n   input: %.3f | %.3f | %.3f" \
        "\n    proc: %.3f | %.3f | %.3f" \
        "\n  output: %.3f | %.3f | %.3f",
        param_.stats_window_sec,
        param_.module_name.c_str(),
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

    if (param_.proc_delay_warn_thr > 0 && output->process_delay_max > param_.proc_delay_warn_thr) {
      RCLCPP_WARN(logger_,
        "process_delay_max [%.3f] exceeds thr [%.3f] in module [%s]",
        output->process_delay_max,
        param_.proc_delay_warn_thr,
        param_.module_name.c_str()
      );

      if (param_.warn2file) {
        if (ofs_log_ && ofs_log_.is_open()) {
          ofs_log_ << msg_ts.sec << "." << msg_ts.nanosec
            << "\t input_delay_max: " << output->input_delay_max
            << "\t process_delay_max: " << output->process_delay_max
            << "\t output_delay_max: " << output->output_delay_max
            << "\n";
        } else {
          RCLCPP_ERROR_ONCE(logger_,
            "[%s] file is not openned in path [%s], write proc delay warn log to file failed, this log appears only once",
            param_.module_name.c_str(),
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

RuntimeStatsErrCode RuntimeStats::TrigerOff(const builtin_interfaces::msg::Time& msg_ts,
  const builtin_interfaces::msg::Time& now_ts) {
  if (!IsEnabled()) {
    RCLCPP_WARN_ONCE(logger_, "Runtime stats is not enabled, this msg appears only once.");
    return RuntimeStatsErrCode::DISABLED;
  }
  std::shared_ptr<RuntimeStatsOutput> output;
  return TrigerOff(msg_ts, now_ts, output);
}

std::string RuntimeStats::GetFullName(std::string param_name) {
  if (!param_.module_name.empty()) {
    return param_.module_name + "." + param_.nm_name + "." + param_name;
  } else {
    return param_.nm_name + "." + param_name;
  }
}

}