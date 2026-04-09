# Semantic-Octomap

`semantic_octomap` builds a probabilistic 3D semantic octree from semantic point cloud input and publishes both 3D and 2D map products for downstream planning and visualization.

## Overview

This repository provides two input pipelines:

1. Image-based semantic point cloud generation  
   `launch/semantic_octomap.launch` starts the original `semantic_sensor_node.py` image pipeline and the octomap node.
2. Generic point cloud to semantic point cloud bridging  
   `launch/pointcloud_semantic_octomap.launch` starts a C++ bridge node that converts an arbitrary point cloud plus synchronized odometry into the semantic point type expected by the octomap node.
3. Global registered point cloud bridging  
   `launch/pointcloud_semantic_octomap_fastlivo.launch` starts the same C++ bridge in cloud-only mode for upstream systems that already publish registered global-frame clouds.

The octomap backend is shared by both pipelines.

## Published Map Topics

`semantic_octomap_node` publishes:

- `octomap_full`: full semantic octree serialization
- `octomap_color`: maximum-likelihood semantic color octree
- `occupancy_map_2D`: 2D occupancy projection

For RViz, prefer `octomap_color`. In practice, `octomap_rviz_plugins` can visualize `octomap_color` reliably, while visualizing `octomap_full` may crash RViz.

## Dependencies

- ROS Melodic or Noetic
- Catkin
- PCL
- OctoMap
- `octomap_msgs`
- `octomap_rviz_plugins`
- `scikit-image`

## Installation

Place this repository in the `src` directory of a catkin workspace, then install dependencies and build:

```bash
cd /path/to/catkin_ws/src
git clone <your-semantic_octomap-repo-url> semantic_octomap
cd ..
rosdep install --from-paths src --ignore-src -r -y
catkin build semantic_octomap
source devel/setup.bash
```

If you are integrating it into an existing workspace, you can also copy or move the repository into `src/` and rebuild with:

```bash
catkin build semantic_octomap
source devel/setup.bash
```

## Quick Start

Build the package:

```bash
catkin build semantic_octomap
source devel/setup.bash
```

Run the original image-based pipeline:

```bash
roslaunch semantic_octomap semantic_octomap.launch
```

Run the generic point cloud bridge pipeline:

```bash
roslaunch semantic_octomap pointcloud_semantic_octomap.launch
```

Run the simulator example:

```bash
roslaunch semantic_octomap examples/simulator_pointcloud_semantic_octomap.launch
```

Run the FAST-LIVO2 example:

```bash
roslaunch semantic_octomap pointcloud_semantic_octomap_fastlivo.launch
```

## Directory Layout

- `launch/`: launch entrypoints for the image-based pipeline, point cloud bridge pipeline, and ready-to-run examples
- `params/`: YAML configuration for bridge behavior and octomap fusion parameters
- `src/semantic_octomap_node/`: ROS node implementations, including the octomap node and point cloud bridge
- `src/semantic_octree/`: semantic octree core data structures and update logic
- `include/`: public headers for the octomap node, octree, and point cloud types
- `scripts/ssmi_sensors/`: Python image-based semantic point cloud generation scripts
- `msg/`, `srv/`: custom ROS messages and services used by the package

## Parameter Docs

Parameter summaries and tuning suggestions are documented separately:

- [README_parameters.md](README_parameters.md)
- [README_parameters_CN.md](README_parameters_CN.md)

## Pipeline 1: Original Image-Based Workflow

Use [semantic_octomap.launch](launch/semantic_octomap.launch) when your semantic point cloud is produced from aligned RGB, depth, and semantic segmentation images.

### Start

```bash
catkin build semantic_octomap
roslaunch semantic_octomap semantic_octomap.launch
```

### Notes

- `semantic_sensor_node.py` generates the semantic point cloud.
- `semantic_octomap_node` consumes that point cloud and updates the semantic octree.
- Camera intrinsics are loaded from `params/semantic_cloud.yaml`.
- Runtime octomap parameters such as `resolution`, `max_range`, and `world_frame_id` are set in the launch file.

## Pipeline 2: Generic Point Cloud Bridge

Use [pointcloud_semantic_octomap.launch](launch/pointcloud_semantic_octomap.launch) when you already have a point cloud source and odometry, for example from a simulator, LiDAR SLAM, or another upstream mapping stack.

### Start

```bash
catkin build semantic_octomap
roslaunch semantic_octomap pointcloud_semantic_octomap.launch
```

### Simulator Example

A ready-to-use simulator example is provided in [simulator_pointcloud_semantic_octomap.launch](launch/examples/simulator_pointcloud_semantic_octomap.launch):

```bash
roslaunch semantic_octomap examples/simulator_pointcloud_semantic_octomap.launch
```

### Override Topics at Launch

```bash
roslaunch semantic_octomap pointcloud_semantic_octomap.launch \
  input_cloud_topic:=/your/cloud \
  input_odom_topic:=/your/odom
```

### Current Default Launch Values

The current defaults in [pointcloud_semantic_octomap.launch](launch/pointcloud_semantic_octomap.launch) are:

- `input_cloud_topic:=/quad0_pcl_render_node/cloud`
- `input_odom_topic:=/quad_0/lidar_slam/odom`
- `output_cloud_topic:=/semantic_pcl/semantic_pcl`
- `world_frame_id:=world`
- `sensor_frame_id:=semantic_sensor`
- `publish_sensor_tf:=true`

### FAST-LIVO2 / Registered Global-Cloud Mode

Use [pointcloud_semantic_octomap_fastlivo.launch](launch/pointcloud_semantic_octomap_fastlivo.launch) when the upstream system already publishes a registered point cloud in a global frame, such as FAST-LIVO2 `/cloud_registered`.

```bash
roslaunch semantic_octomap pointcloud_semantic_octomap_fastlivo.launch
```

This launch is intentionally different from the default bridge launch:

- it uses `world_frame_id:=camera_init`
- it keeps the incoming global cloud frame
- it defaults `publish_sensor_tf:=false`
- it disables odometry synchronization with `use_odom_sync:=false`

This is useful for registered map-frame clouds whose point cloud and odometry timestamps are not guaranteed to match closely enough for message filter synchronization.

## Generic Bridge Behavior

The C++ bridge node is implemented in [pointcloud_to_semantic_cloud_node.cpp](src/semantic_octomap_node/pointcloud_to_semantic_cloud_node.cpp). It supports two operating modes:

- synchronized cloud + odometry mode
- cloud-only mode for already registered global-frame clouds

In both cases it converts the input to `PointXYZRGBSemantic` and publishes the semantic point cloud consumed by the octomap node.

### TF Publishing Control

The bridge also supports a runtime `publish_sensor_tf` switch exposed by the launch files:

- `publish_sensor_tf:=true` keeps the original behavior and broadcasts `world_frame_id -> sensor_frame_id` during synchronized cloud + odometry processing
- `publish_sensor_tf:=false` disables that TF publication

This is mainly useful when the upstream system already provides the needed TF, or when the bridge is being used in cloud-only mode with already registered global-frame clouds.

The bridge also exposes `use_odom_sync` through the launch files:

- `use_odom_sync:=true` enables synchronized cloud + odometry processing
- `use_odom_sync:=false` switches the bridge into cloud-only mode

This is useful for registered global-frame clouds whose output no longer needs per-cloud odometry pairing.

### Frame Handling

The bridge supports both common upstream cloud conventions:

- If the input cloud is already in `world_frame_id`, the bridge preserves that frame to avoid applying odometry twice.
- If the input cloud is in the sensor-local frame, the bridge publishes it in `sensor_frame_id` and lets the octomap node transform it through TF.
- If odometry synchronization is disabled, the bridge keeps the input cloud frame directly and does not publish sensor TF.

This prevents the common “map appears higher or shifted than the cloud” issue caused by double transformation.

### Attribute Reuse

When the input cloud already contains color or semantic attributes, the bridge reuses them instead of overwriting them with defaults.

Supported reusable fields:

- `rgb`
- `rgba`
- split `r`, `g`, `b`
- `semantic_color`

If these fields are missing, the bridge falls back to the default colors from [pointcloud_semantic_bridge.yaml](params/pointcloud_semantic_bridge.yaml).

### Extra Fields

Fields not used by the bridge, such as `intensity`, `ring`, or `time`, do not break the pipeline. They are ignored during conversion. The output semantic cloud only keeps:

- `x`
- `y`
- `z`
- `rgb`
- `semantic_color`

### Coordinate Field Requirement

The bridge currently requires:

- `x`
- `y`
- `z`

to exist as scalar `FLOAT32` fields in the incoming `sensor_msgs/PointCloud2`.

If `x/y/z` are missing or use an unsupported datatype, the bridge will log an error and drop that cloud instead of silently decoding wrong coordinates.

## Launch and Parameter Split

The current configuration is intentionally split this way:

- [pointcloud_semantic_octomap.launch](launch/pointcloud_semantic_octomap.launch) holds runtime wiring such as topics, frames, map resolution, range, and save path.
- [pointcloud_semantic_bridge.yaml](params/pointcloud_semantic_bridge.yaml) holds bridge-internal parameters such as sync behavior and default fallback colors.
- [octomap_generator.yaml](params/octomap_generator.yaml) holds octomap fusion internals such as hit/miss probabilities and clamping thresholds.

## RViz Notes

- Set RViz `Fixed Frame` to match `world_frame_id`.
- Add an `Octomap` display and subscribe to `octomap_color`.
- Use `occupancy_map_2D` for the 2D map view.
- If the map looks vertically shifted relative to the cloud, first check whether the input cloud is already in `world` frame and whether the odometry frame corresponds to the actual sensor frame.

## Files You Will Most Likely Touch

- [launch/pointcloud_semantic_octomap.launch](launch/pointcloud_semantic_octomap.launch)
- [launch/examples/simulator_pointcloud_semantic_octomap.launch](launch/examples/simulator_pointcloud_semantic_octomap.launch)
- [params/pointcloud_semantic_bridge.yaml](params/pointcloud_semantic_bridge.yaml)
- [params/octomap_generator.yaml](params/octomap_generator.yaml)
- [src/semantic_octomap_node/pointcloud_to_semantic_cloud_node.cpp](src/semantic_octomap_node/pointcloud_to_semantic_cloud_node.cpp)

## Build

```bash
catkin build semantic_octomap
```

## Acknowledgements

`semantic_octomap` is being released as a standalone repository based on work originally developed in the broader SSMI project.

Sincere thanks to the authors and maintainers of the open-source SSMI repository for making the original system publicly available:

- [ExistentialRobotics/SSMI](https://github.com/ExistentialRobotics/SSMI)

If this repository is useful in your work, please also acknowledge the original SSMI project.
