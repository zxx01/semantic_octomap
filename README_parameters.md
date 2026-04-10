# Semantic Octomap Parameter Guide

This document summarizes the adjustable parameters currently used by the `semantic_octomap` pipeline.
The main [README.md](README.md) focuses on workflow and integration, while this document focuses on defaults and tuning.

## Parameter Priority

At runtime, parameter priority is:

1. launch arguments in the active launch file
2. YAML defaults in `params/octomap_generator.yaml` and `params/pointcloud_semantic_bridge.yaml`
3. hard-coded fallback values in C++ source

In practice:

- topic and frame settings are controlled by the launch file
- bridge behavior defaults are mainly controlled by `pointcloud_semantic_bridge.yaml`
- octomap fusion behavior is controlled by `octomap_generator.yaml`
- octomap runtime settings such as resolution, range, frame, and save path are controlled by launch files

## 1. Point Cloud Bridge YAML Parameters

These parameters come from `params/pointcloud_semantic_bridge.yaml` and control the internal behavior of the generic point-cloud-to-semantic-point-cloud bridge node.
Launch files can still override bridge behavior at runtime, such as `publish_sensor_tf`.

| Parameter | Default | Meaning | Tuning suggestion |
| --- | --- | --- | --- |
| `sync_queue_size` | `10` | Queue size for approximate time synchronization between cloud and pose. | Increase if messages drop under bursty input. Decrease if you want lower latency and your timestamps are stable. |
| `sync_slop` | `0.05` | Maximum allowed timestamp mismatch, in seconds, for cloud-pose pairing. | If synchronization fails often, increase slightly such as `0.08` or `0.1`. If wrong poses are being paired to clouds, decrease it. |
| `rgb_bgr` | `[180, 180, 180]` | Fallback RGB color assigned to each bridged point when the input cloud does not already provide reusable color fields. | Mainly affects visualization. If the upstream cloud already carries `rgb`, `rgba`, or `r/g/b`, that input is reused and this value is ignored. |
| `semantic_bgr` | `[0, 255, 0]` | Fallback semantic color assigned to each bridged point when the input cloud does not already provide `semantic_color`. | This defines the default single semantic class for generic unlabelled clouds. If the upstream cloud already carries `semantic_color`, that input is reused and this value is ignored. |

## 2. Semantic Octomap Core Parameters

These parameters control how occupancy and semantic evidence are fused into the octree. They are defined in `params/octomap_generator.yaml`.

| Parameter | Default | Meaning | Tuning suggestion |
| --- | --- | --- | --- |
| `octomap/clamping_thres_min` | `0.0001` | Lower clamp bound for occupancy and semantic log-odds updates. | Raise it a little if you want old evidence to decay less aggressively. Keep it low if free-space clearing should stay strong. |
| `octomap/clamping_thres_max` | `0.9999` | Upper clamp bound for occupancy and semantic log-odds updates. | Lower it slightly if occupied cells become too hard to overturn after occasional false positives. |
| `octomap/occupancy_thres` | `0.5` | Occupancy decision threshold for considering a voxel occupied. | Increase above `0.5` for more conservative obstacle marking. Decrease slightly for faster occupancy confirmation. |
| `octomap/prob_hit` | `0.7` | Occupancy update probability when a point is observed as occupied. | Increase if true obstacle returns are stable and you want faster convergence. Decrease if your point cloud has more spurious hits. |
| `octomap/prob_miss` | `0.4` | Occupancy update probability when a ray passes through free space. | Lower values clear space more gently. If stale occupied cells remain too long, move this closer to `0.45`. |
| `octomap/psi` | `1` | Positive semantic log-odds increment for the observed semantic class. | Increase if you want the observed semantic label to dominate faster. If labels are noisy, reduce it. |
| `octomap/phi` | `-0.1` | Negative semantic log-odds increment applied to non-observed classes and free updates. | More negative values make semantics change faster and clear old beliefs faster. If semantics flip too easily, move it closer to `0`. |
| `octomap/publish_2d_map` | `true` | Whether to publish the projected 2D occupancy map. | Disable it if you only need the 3D octomap and want to save some runtime overhead. |

## 3. Launch-Level Runtime Arguments

These are the main arguments exposed by `launch/pointcloud_semantic_octomap.launch`. They control runtime integration and octomap deployment settings.

| Parameter | Default | Meaning | Tuning suggestion |
| --- | --- | --- | --- |
| `input_cloud_topic` | `/quad0_pcl_render_node/cloud` | Launch-level override for the bridge input cloud topic. | Use this for fast integration with a new sensor or simulator without editing YAML. If your upstream cloud is already in `world_frame_id`, the bridge will preserve that frame automatically. |
| `input_odom_topic` | `/quad_0/lidar_slam/odom` | Launch-level override for the bridge input odometry topic. | Keep it synchronized with `input_cloud_topic`. If odometry is for `base_link` while the cloud is from a displaced lidar frame, you may still need an explicit sensor extrinsic. |
| `output_cloud_topic` | `/semantic_pcl/semantic_pcl` | Launch-level override for the bridge output cloud topic. | Useful if multiple semantic mapping pipelines share a ROS master. |
| `world_frame_id` | `world` | Launch-level override for the global mapping frame. | Keep consistent across bridge, TF, and octomap. |
| `sensor_frame_id` | `semantic_sensor` | Launch-level override for the sensor TF child frame. | Change only when integrating with an existing frame naming convention. |
| `publish_sensor_tf` | `true` | Whether the bridge should broadcast `world_frame_id -> sensor_frame_id` TF during synchronized cloud + odometry processing. | Disable it when the upstream system already publishes the needed TF, or when you want the bridge to stay TF-neutral. |
| `display_color_mode` | `semantic` | Default serialization color mode for the `octomap_color` topic. Accepted values are `semantic` and `rgb`. | Use `semantic` when RViz should show semantic class colors. Use `rgb` when you want the map to reflect input RGB colors instead. The runtime `toggle_use_semantic_color` service can switch this later without restarting the node. |
| `rviz` | `true` | Whether to start RViz together with the generic bridge pipeline. | Keep it on for bring-up and debugging, or disable it on heavier runs when you do not need live visualization. |
| `octomap_resolution` | `0.4` | Launch override for `octomap/resolution`. | This is usually one of the first parameters to tune for map quality versus runtime. |
| `octomap_max_range` | `15.0` | Launch override for `octomap/max_range`. | Tune together with sensor range and noise level. |
| `octomap_raycast_range` | `15.0` | Launch override for `octomap/raycast_range`. | Usually keep equal to or slightly below `octomap_max_range`. |
| `min_ground_z` | `1.0` | Launch override for `octomap/min_ground_z`. | Useful for quick 2D occupancy projection tuning without touching YAML. |
| `max_ground_z` | `3.5` | Launch override for `octomap/max_ground_z`. | Useful for quick 2D occupancy projection tuning without touching YAML. |
| `save_path` | `/tmp/semantic_map.ot` | Launch override for `octomap/save_path`. | Set a unique experiment path if you want to preserve each run. |

Notes:

- In `launch/semantic_octomap.launch`, the octomap node consumes a semantic point cloud directly from the image-based `semantic_sensor_node.py`.
- In `launch/pointcloud_semantic_octomap.launch`, the octomap input topic follows `output_cloud_topic`.

## 4. FAST-LIVO2 Launch Differences

`launch/pointcloud_semantic_octomap_fastlivo.launch` is a ready-to-use launch preset for wiring FAST-LIVO2 `/cloud_body` and `/aft_mapped_to_init` into the same synchronized bridge pipeline.

| Parameter | Default | Meaning | Tuning suggestion |
| --- | --- | --- | --- |
| `rviz` | `false` | Whether to start RViz together with the mapping pipeline. | Keep it off by default on heavier SLAM runs; enable only when you need interactive visualization. |
| `input_cloud_topic` | `/cloud_body` | FAST-LIVO2 point cloud topic. | Keep it aligned with the actual cloud topic published by your FAST-LIVO2 setup. |
| `input_odom_topic` | `/aft_mapped_to_init` | FAST-LIVO2 odometry topic. | Keep it time-aligned with the cloud topic because the bridge now always runs in synchronized cloud + odometry mode. |
| `world_frame_id` | `camera_init` | Global frame used by the FAST-LIVO2 launch preset. | This should normally stay aligned with the upstream cloud frame. |
| `sensor_frame_id` | `semantic_sensor` | TF child frame name used by the bridge. | Change only if you need to match an existing frame naming convention. |
| `publish_sensor_tf` | `true` | Whether the bridge should broadcast `camera_init -> semantic_sensor`. | Disable it when the upstream system already publishes the needed TF and you want to avoid duplicate transforms. |
| `display_color_mode` | `rgb` | Default serialization color mode for the FAST-LIVO2 `octomap_color` topic. Accepted values are `semantic` and `rgb`. | `rgb` is a practical default when you want RViz to follow the registered cloud appearance directly. Switch to `semantic` when semantic class colors are more useful. |
| `octomap_max_range` | `40.0` | FAST-LIVO2 launch override for `octomap/max_range`. | Reduce it if far-range registered points add too much noise or runtime overhead. |
| `octomap_raycast_range` | `40.0` | FAST-LIVO2 launch override for `octomap/raycast_range`. | Usually keep it consistent with `octomap_max_range` unless free-space clearing is too aggressive. |

## 5. Practical Tuning Order

When bringing up a new sensor or simulator, a good tuning order is:

1. `input_cloud_topic`, `input_odom_topic`, `world_frame_id`
2. `sync_slop`, `sync_queue_size`
3. `octomap_resolution`
4. `octomap_max_range`, `octomap_raycast_range`
5. `prob_hit`, `prob_miss`, `occupancy_thres`
6. `psi`, `phi`
7. `min_ground_z`, `max_ground_z`

## 6. Quick Recipes

| Goal | Suggested adjustment |
| --- | --- |
| Map too coarse | Decrease `octomap_resolution` to `0.1` or `0.15`. |
| Runtime or memory too high | Increase `octomap_resolution` to `0.3` or `0.5`, and consider reducing `octomap_max_range`. |
| Free-space clearing too weak | Increase `octomap_raycast_range` or move `prob_miss` slightly upward. |
| False obstacles persist too long | Lower `clamping_thres_max` slightly or increase the free-space effect by adjusting `prob_miss`. |
| Semantic labels change too slowly | Increase `psi` or make `phi` more negative. |
| Semantic labels flicker | Decrease `psi` or move `phi` closer to `0`. |
| 2D occupancy map includes too much vertical clutter | Reduce `max_ground_z` or increase `min_ground_z`. |
