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

#include <memory>
#include <queue>
#include <cfloat>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

#ifndef TROS_RUNTIME_STATS_HPP_
#define TROS_RUNTIME_STATS_HPP_

namespace tros {
struct RuntimeStatsParams {
  std::string module_name = "";
  std::string nm_name = "tros_perf";
  // The params will be parsed with module_name.nm_name.[name]
  bool enabled = true;
  bool print_stat = true;
  // If enable_debug is true, the debug info will be printed with info log level
  bool enable_debug = false;
  float stats_window_sec = 5.0;
  int stats_cache_len_thr = 150;
  float msg_cache_timeout_thr = 1.0;
  int msg_cache_len_thr = 10;

  // If max of process delay exceeds proc_delay_warn_thr, output warnning info
  // Enabled if proc_delay_warn_thr > 0
  float proc_delay_warn_thr = -1.0;
  // Whether output warnning info to file
  // file name: file_path/[module_name]_[nm_name]_[stamp.sec].log
  bool warn2file = true;
  std::string file_path = "./";

  RuntimeStatsParams() {}
  RuntimeStatsParams(std::string _module_name, bool _enabled = false) :
    module_name(_module_name), enabled(_enabled) {}
};

struct RuntimeStatsOutput {
  float input_delay_min = FLT_MAX;
  float input_delay_max = FLT_MIN;
  float input_delay_avg = 0;

  float process_delay_min = FLT_MAX;
  float process_delay_max = FLT_MIN;
  float process_delay_avg = 0;

  float output_delay_min = FLT_MAX;
  float output_delay_max = FLT_MIN;
  float output_delay_avg = 0;

  float output_fps;
};

enum class RuntimeStatsErrCode {
  SUCCESS = 0,
  DISABLED,
  INVALID_STAMP,
  UNKNOWN
};

struct RuntimeFrameStat;

template <typename NodeWeakPtrType>
class RuntimeStats {
 public:
  RuntimeStats(const NodeWeakPtrType & parent,
    RuntimeStatsParams param = RuntimeStatsParams());

  ~RuntimeStats();

  RuntimeStatsErrCode TrigerOn(const builtin_interfaces::msg::Time& msg_ts,
    const builtin_interfaces::msg::Time& now_ts);
  RuntimeStatsErrCode TrigerOff(const builtin_interfaces::msg::Time& msg_ts,
    const builtin_interfaces::msg::Time& now_ts,
    std::shared_ptr<RuntimeStatsOutput>& output);
  RuntimeStatsErrCode TrigerOff(const builtin_interfaces::msg::Time& msg_ts,
    const builtin_interfaces::msg::Time& now_ts);

  bool IsEnabled();

 private:
  void Init(builtin_interfaces::msg::Time stamp);
  RuntimeStatsParams param_;
  rclcpp::Logger logger_{rclcpp::get_logger("tros")};
  std::map<double, RuntimeFrameStat> msg_cache_;
  std::queue<RuntimeFrameStat> stats_cache_;
  std::mutex stat_mtx_;
  std::ofstream ofs_log_;

 private:
  std::string GetFullName(std::string);

  template <
    typename NodeType,
    typename ParamType>
  void ParseParam(NodeType node, const std::string& param_name, ParamType& param) {
    if (!node->has_parameter(GetFullName(param_name))) {
      node->declare_parameter(GetFullName(param_name), param);
    }
    node->get_parameter(GetFullName(param_name), param);
  }

  template <typename NodeType>
  void ParseParams(NodeType node) {
    ParseParam(node, "enabled", param_.enabled);
    ParseParam(node, "print_stat", param_.print_stat);
    ParseParam(node, "enable_debug", param_.enable_debug);
    ParseParam(node, "stats_window_sec", param_.stats_window_sec);
    ParseParam(node, "stats_cache_len_thr", param_.stats_cache_len_thr);
    ParseParam(node, "msg_cache_timeout_thr", param_.msg_cache_timeout_thr);
    ParseParam(node, "msg_cache_len_thr", param_.msg_cache_len_thr);
    ParseParam(node, "proc_delay_warn_thr", param_.proc_delay_warn_thr);
    ParseParam(node, "warn2file", param_.warn2file);
    ParseParam(node, "file_path", param_.file_path);
  }

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterEventCallbackHandle> event_cb_handle_;
  void AddParamCallback();
  void PrintParam();
};
}

#endif // !TROS_RUNTIME_STATS_HPP_