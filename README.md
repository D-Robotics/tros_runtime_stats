# 功能

用于统计处理延迟，包括输出的帧率，统计时间窗口内的平均、最大和最小延迟。

# 使用

在代码中创建`RuntimeStats`对象，使用其接口传入时间戳，即可统计延迟。当统计时间窗口满时，会打印统计结果。

```c++
// 1. Include "runtime_stats.h"
#include "tros_runtime_stats/runtime_stats.h"

// 2. Construct an instance
// Pass param with module_name and node ptr
auto sp_runtime_stat_ = 
  std::make_shared<tros::RuntimeStats>(shared_from_this(), tros::RuntimeStatsParams(module_name));

// 3. Triger with time stamp in callback
// When callback start, triger on with time stamp of msg and now
sp_runtime_stat_->TrigerOn(msg->header.stamp, this->now());
// Do something in callback
// When callback end, triger off with time stamp of msg and now
sp_runtime_stat_->TrigerOff(msg->header.stamp, this->now());
// The perf will be output to console if the time interval is longer than time window
// Pass and get the output in 'TrigerOff' API if you need it
std::shared_ptr<RuntimeStatsOutput> output;
sp_runtime_stat_->TrigerOff(msg->header.stamp, this->now(), output);
```

# [sample](./sample/main.cpp)示例

- 终端1运行消息发布：

```bash
# ros2 topic pub -r 2 /chatter std_msgs/msg/String "{data: 'hello'}"
```

- 终端2运行消息订阅：

```bash
# ros2 run tros_runtime_stats tros_runtime_stats_sample
```

- 运行后终端2输出：

```bash
[WARN] [1755658910.424465633] [sub_node]: Runtime stats in module [demo] is enabled
[WARN] [1755658910.424968397] [sub_node]:
           module_name: demo
               nm_name: tros_perf
               enabled: true
            print_stat: true
          enable_debug: false
      stats_window_sec: 1.00
   stats_cache_len_thr: 150
 msg_cache_timeout_thr: 1.00
     msg_cache_len_thr: 10
   proc_delay_warn_thr: -1.000
             warn2file: true
             file_path: ./
[INFO] [1755658910.480940013] [sub_node]: I heard: 'hello'
[INFO] [1755658910.481284952] [sub_node]: sleep 83 ms
[INFO] [1755658910.981132929] [sub_node]: I heard: 'hello'
[INFO] [1755658910.981371290] [sub_node]: sleep 86 ms
[INFO] [1755658911.481048902] [sub_node]: I heard: 'hello'
[INFO] [1755658911.481384841] [sub_node]: sleep 77 ms
[WARN] [1755658911.558768576] [sub_node]: perf in window [1.00] sec:
  module: demo
 out fps: 3.00
   delay: min   | max   | avg
   input: 0.000 | 0.000 | 0.000
    proc: 0.077 | 0.086 | 0.082
  output: 0.077 | 0.086 | 0.082
[INFO] [1755658911.980826633] [sub_node]: I heard: 'hello'
[INFO] [1755658911.981012414] [sub_node]: sleep 15 ms
[INFO] [1755658912.480874016] [sub_node]: I heard: 'hello'
[INFO] [1755658912.481050922] [sub_node]: sleep 93 ms
[INFO] [1755658912.981259463] [sub_node]: I heard: 'hello'
[INFO] [1755658912.981446952] [sub_node]: sleep 35 ms
[WARN] [1755658913.016706285] [sub_node]: perf in window [1.00] sec:
  module: demo
 out fps: 3.00
   delay: min   | max   | avg
   input: 0.000 | 0.000 | 0.000
    proc: 0.015 | 0.093 | 0.048
  output: 0.015 | 0.093 | 0.048
```

# 参数

# 参数说明

| 参数名称 | 参数类型 | 参数作用 | 参数默认值 |
| :--- | :--- | :--- | :--- |
| module_name | string | 用于统计的模块名，例如"local_costmap" | "" |
| nm_name     | string | 统计节点的namespace，例如"tros_perf" | "tros_perf" |
| enabled     | bool   | 是否开启统计功能 | false |
| print_stat  | bool   | 是否打印统计结果到终端 | true |
| enable_debug | bool   | 是否输出nfo log level的debug信息 | false |
| stats_window_sec | float | 统计窗口大小，单位为秒 | 5.0 |
| stats_cache_len_thr | int | 缓存长度阈值，当缓存长度超过该阈值时，将删除缓存中最早的消息 | 150 |
| msg_cache_timeout_thr | float | 消息缓存超时阈值，单位为秒 | 1.0 |
| proc_delay_warn_thr | float | 处理延迟阈值，单位为秒，大于0有效，超过阈值时输出警告信息 | -1.0 |
| warn_file | bool | 是否将处理超过阈值的警告信息保存到文件，文件名为`file_path/[module_name]_[nm_name]_[stamp.sec].log` | true |
| file_path | string | 保存警告信息的文件路径 | "./" |

# 在yaml配置文件中设置参数

示例：

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath", "TrosLocalPlanner"]
    TrosLocalPlanner:
      tros_perf:
        enabled: True
        print_stat: True
        enable_debug: False # info log level
        stats_window_sec: 5.0
        stats_cache_len_thr: 150
        msg_cache_timeout_thr: 1.0
        msg_cache_len_thr: 10
        proc_delay_warn_thr: 0.05
        warn2file: true
        file_path: "./perf"
```

其中`TrosLocalPlanner`为`module_name`，`tros_perf`为`nm_name`。
