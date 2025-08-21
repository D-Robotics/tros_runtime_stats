# 功能

统计时间窗口内的运行时状态，具体功能列表：

- 统计模块的输出帧率；
- 统计输入、处理和输出延迟，包括时间窗口内的平均、最大和最小延迟；
- 支持指定统计的窗口时间；
- 支持指定异常延迟阈值，以及将异常延迟信息保存到本地文件中；
- 支持`debug`模式，用于输出更丰富的运行时状态信息；
- 支持动态开关`debug`模式；
- 支持动态开关统计功能；
- 支持使用`ros2 param get/set`命令动态查询/修改统计参数；

# 使用

在代码中创建`RuntimeStats`的模板类对象，在消息回调中使用其接口传入时间戳，即可统计延迟。当统计时间窗口满时，打印统计结果。

注意：

- `RuntimeStats`模板类只支持`rclcpp::Node::WeakPtr`和`rclcpp_lifecycle::LifecycleNode::WeakPtr`两种类型特化。

- `RuntimeStats`模板类对象创建时，需要传入节点指针，用于`RuntimeStats`内部更新配置参数。

- 当使用`rclcpp::Node::WeakPtr`类型特化时，并且在节点构造函数中创建`RuntimeStats`对象，必须通过**延迟构造**的方式创建`RuntimeStats`对象，避免传入的节点指针无效。例如在Node构造函数中使用线程异步延迟创建`RuntimeStats`对象：
          
  ```c++
    std::thread([this]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      auto param = tros::RuntimeStatsParams(
        this->get_node_base_interface()->get_name(), true);
      param.stats_window_sec = 1.0;
      sp_runtime_stat_ = 
        std::make_shared<tros::RuntimeStats<rclcpp::Node::WeakPtr>>(shared_from_this(), param);
    }).detach();
  ```

  除了使用线程，也可以使用`ros2 timer`实现延迟构造。

- `RuntimeStats`提供了`TrigerOn`和`TrigerOff`两个接口实现统计，调用的时间点是分别消息回调开始和结束，传递的接口两个参数分别是消息的时间戳和系统当前时间。

# 原理

- 输出帧率

  帧率 = 窗口时间内处理完的消息数 / 窗口时间

- 输入延迟

  输入延迟 = `TrigerOn`接口传入的系统当前时间 - 消息时间戳中的时间

- 处理延迟

  处理延迟 = 消息处理结束时间（`TrigerOff`接口传入的系统时间） - 消息处理开始时间（`TrigerOn`接口传入的系统时间）

- 输出延迟

  输入延迟 = `TrigerOff`接口传入的系统当前时间 - 消息时间戳中的时间

# sample示例

[示例代码](./sample/main.cpp)。

- 终端1运行消息发布：

```bash
ros2 topic pub -r 2 /chatter std_msgs/msg/String "{data: 'hello'}"
```

- 终端2运行消息订阅：

```bash
ros2 run tros_runtime_stats tros_runtime_stats_sample
```

- 运行后终端2输出：

```bash
[WARN] [1755658910.424465633] [sub_node]: Runtime stats in module [sub_node] is enabled
[WARN] [1755658910.424968397] [sub_node]:
           node_name: sub_node
               nm_name: tros_perf
               enabled: true
            print_stat: true
          enable_debug: false (warn log level)
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
  module: sub_node
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
  module: sub_node
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
| node_name | string | 用于统计的模块名，例如"local_costmap" | "" |
| nm_name     | string | 统计节点的namespace，例如"tros_perf" | "tros_perf" |
| enabled     | bool   | 是否开启统计功能 | true |
| print_stat  | bool   | 是否打印统计结果到终端 | true |
| enable_debug | bool   | 是否输出nfo log level的debug信息 | false |
| stats_window_sec | float | 统计窗口大小，单位为秒 | 5.0 |
| stats_cache_len_thr | int | 缓存长度阈值，当缓存长度超过该阈值时，将删除缓存中最早的消息 | 150 |
| msg_cache_timeout_thr | float | 消息缓存超时阈值，单位为秒 | 1.0 |
| proc_delay_warn_thr | float | 处理延迟阈值，单位为秒，大于0有效，超过阈值时输出警告信息 | -1.0 |
| warn_file | bool | 是否将处理超过阈值的警告信息保存到文件，文件名为`file_path/[node_name]_[nm_name]_[stamp.sec].log` | true |
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

其中`TrosLocalPlanner`为`node_name`，`tros_perf`为`nm_name`。
