# 语义 Octomap 参数说明

本文档整理了 `semantic_octomap` 当前语义八叉树建图链路中可调的参数。主 [README.md](README.md) 侧重说明整体工作流和接入方式，这份文档侧重默认值和调参建议。

## 参数优先级

运行时参数优先级如下：

1. 当前启动所使用 launch 文件中的参数
2. `params/octomap_generator.yaml` 与 `params/pointcloud_semantic_bridge.yaml` 中的 YAML 默认值
3. C++ 源码中的硬编码兜底默认值

实际使用时可以这样理解：

- 话题名和坐标系相关参数由 launch 文件控制
- 点云桥接节点本身的行为参数主要由 `pointcloud_semantic_bridge.yaml` 控制
- 八叉树融合行为参数由 `octomap_generator.yaml` 控制
- 八叉树运行时参数，如分辨率、量程、frame、保存路径，由 launch 文件控制

## 1. 点云桥接 YAML 参数

这些参数来自 `params/pointcloud_semantic_bridge.yaml`，用于控制“原始点云 -> 语义点云”的通用桥接节点内部行为。

| 参数名 | 默认值 | 含义 | 调节建议 |
| --- | --- | --- | --- |
| `sync_queue_size` | `10` | 点云与位姿近似时间同步器的队列长度。 | 如果输入消息有突发、掉同步，可适当调大；如果更关注低延迟且时间戳稳定，可适当调小。 |
| `sync_slop` | `0.05` | 点云与位姿允许匹配的最大时间差，单位秒。 | 如果经常同步不上，可略微增大到 `0.08` 或 `0.1`；如果错误配对较多，则应减小。 |
| `rgb_bgr` | `[180, 180, 180]` | 当输入点云本身没有可复用颜色字段时，桥接节点使用的兜底 RGB 颜色。 | 主要影响可视化显示。如果上游点云已经带有 `rgb`、`rgba` 或 `r/g/b`，则会优先沿用输入值，这个参数不会生效。 |
| `semantic_bgr` | `[0, 255, 0]` | 当输入点云本身没有 `semantic_color` 字段时，桥接节点使用的兜底语义颜色。 | 对未标注点云来说，它定义了默认的单一语义类别。如果上游点云已经带有 `semantic_color`，则会优先沿用输入值，这个参数不会生效。 |

## 2. 语义八叉树核心参数

这些参数控制占据概率和语义证据是如何融合进八叉树的，它们定义在 `params/octomap_generator.yaml` 中。

| 参数名 | 默认值 | 含义 | 调节建议 |
| --- | --- | --- | --- |
| `octomap/clamping_thres_min` | `0.0001` | 占据与语义 log-odds 更新的下限截断值。 | 如果你希望旧证据不要被太快削弱，可以适当调大；如果希望空闲空间清除能力更强，则保持较小值。 |
| `octomap/clamping_thres_max` | `0.9999` | 占据与语义 log-odds 更新的上限截断值。 | 如果误检障碍一旦写入后很难被修正，可以适当减小。 |
| `octomap/occupancy_thres` | `0.5` | 判断一个体素是否占据的阈值。 | 大于 `0.5` 会更保守，不容易把点判成障碍；适当降低会让占据更快成立。 |
| `octomap/prob_hit` | `0.7` | 观测到障碍点时的占据更新概率。 | 如果点云命中稳定、希望更快收敛，可调大；如果噪点较多，应适当减小。 |
| `octomap/prob_miss` | `0.4` | 射线穿过空闲区域时的占据更新概率。 | 数值更低时，空闲清除会更温和；如果旧障碍残留太久，可略微增大到接近 `0.45`。 |
| `octomap/psi` | `1` | 对“当前观测到的语义类别”施加的正向语义 log-odds 增量。 | 如果希望当前语义类别更快占主导，可调大；如果语义观测噪声较大，则应减小。 |
| `octomap/phi` | `-0.1` | 对“未观测到的其它类别”以及 free 更新施加的负向语义 log-odds 增量。 | 数值越负，旧语义会被更快削弱、语义切换更快；如果语义容易抖动，可把它调得更接近 `0`。 |
| `octomap/publish_2d_map` | `true` | 是否发布投影后的 2D 占据栅格地图。 | 如果你只关心 3D 八叉树，可以关闭它以减少一些运行开销。 |

## 3. Launch 层运行参数

这些参数是在 `launch/pointcloud_semantic_octomap.launch` 中暴露出来的主要启动参数，用于控制运行时接线与 octomap 部署参数。

| 参数名 | 默认值 | 含义 | 调节建议 |
| --- | --- | --- | --- |
| `input_cloud_topic` | `/quad0_pcl_render_node/cloud` | launch 层对桥接节点输入点云话题的覆盖。 | 接入新传感器或新仿真器时，优先通过它快速改线，无需动 YAML。如果输入点云本来就在 `world_frame_id` 下，桥接节点会自动保留该 frame，避免重复变换。 |
| `input_odom_topic` | `/quad_0/lidar_slam/odom` | launch 层对桥接节点输入里程计话题的覆盖。 | 必须与 `input_cloud_topic` 时间对齐。如果里程计对应的是 `base_link`，而点云来自有外参偏移的激光雷达，仍可能需要额外的传感器外参。 |
| `output_cloud_topic` | `/semantic_pcl/semantic_pcl` | launch 层对桥接节点输出语义点云话题的覆盖。 | 多条建图链路共存时比较有用。 |
| `world_frame_id` | `world` | launch 层对全局坐标系的覆盖。 | 需要在桥接节点、TF 和 octomap 三者之间保持一致。 |
| `sensor_frame_id` | `semantic_sensor` | launch 层对传感器 TF 子坐标系名称的覆盖。 | 只有在现有系统已经有固定 frame 命名时才建议改。 |
| `octomap_resolution` | `0.4` | 对 `octomap/resolution` 的 launch 覆盖。 | 这是最常调的参数之一，直接决定地图精度与运行代价。 |
| `octomap_max_range` | `15.0` | 对 `octomap/max_range` 的 launch 覆盖。 | 一般结合传感器量程与远距离噪声一起调。 |
| `octomap_raycast_range` | `10.0` | 对 `octomap/raycast_range` 的 launch 覆盖。 | 通常与 `octomap_max_range` 相同或略小。 |
| `min_ground_z` | `1.0` | 对 `octomap/min_ground_z` 的 launch 覆盖。 | 适合快速调 2D 投影范围，不用改 YAML。 |
| `max_ground_z` | `3.5` | 对 `octomap/max_ground_z` 的 launch 覆盖。 | 适合快速调 2D 投影范围，不用改 YAML。 |
| `save_path` | `/tmp/semantic_map.ot` | 对 `octomap/save_path` 的 launch 覆盖。 | 建议为每次实验设定独立路径，便于结果留存。 |

补充说明：

- 在 `launch/semantic_octomap.launch` 中，八叉树节点直接订阅由 `semantic_sensor_node.py` 生成的语义点云。
- 在 `launch/pointcloud_semantic_octomap.launch` 中，八叉树输入点云话题默认跟随 `output_cloud_topic`。

## 4. 推荐调参顺序

当你接入新的传感器或仿真器时，建议按下面顺序调：

1. `input_cloud_topic`、`input_odom_topic`、`world_frame_id`
2. `sync_slop`、`sync_queue_size`
3. `octomap_resolution`
4. `octomap_max_range`、`octomap_raycast_range`
5. `prob_hit`、`prob_miss`、`occupancy_thres`
6. `psi`、`phi`
7. `min_ground_z`、`max_ground_z`

## 5. 常见调参场景

| 目标 | 建议调整 |
| --- | --- |
| 地图太粗 | 将 `octomap_resolution` 降到 `0.1` 或 `0.15`。 |
| 运行太慢或内存占用太高 | 将 `octomap_resolution` 提高到 `0.3` 或 `0.5`，同时考虑减小 `octomap_max_range`。 |
| 空闲空间清除不够明显 | 增大 `octomap_raycast_range`，或适当增大 `prob_miss` 的作用。 |
| 误检障碍残留太久 | 适当减小 `clamping_thres_max`，或者增强 free 更新效果，例如调 `prob_miss`。 |
| 语义类别收敛太慢 | 增大 `psi`，或者让 `phi` 更负一些。 |
| 语义标签抖动明显 | 减小 `psi`，或者把 `phi` 调得更接近 `0`。 |
| 2D 占据图混入太多垂直方向杂物 | 减小 `max_ground_z`，或者增大 `min_ground_z`。 |
