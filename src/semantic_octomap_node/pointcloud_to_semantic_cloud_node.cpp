#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <semantic_point_cloud/semantic_point_type.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

/**
 * @brief Bridge a raw point cloud stream to the semantic point cloud format
 * expected by semantic_octomap.
 *
 * This node synchronizes an input point cloud with the corresponding odometry,
 * optionally broadcasts the sensor pose as TF, and republishes the input cloud
 * with fixed RGB and semantic color fields.
 */
class PointCloudSemanticBridgeNode
{
public:
  /**
   * @brief Construct the bridge node and initialize ROS publishers,
   * subscribers, and parameters.
   *
   * @param nh Global node handle used for topic advertisement and
   * subscription.
   * @param private_nh Private node handle used for bridge parameters.
   */
  PointCloudSemanticBridgeNode(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
      : nh_(nh), private_nh_(private_nh)
  {
    private_nh_.param<std::string>("input_cloud_topic", input_cloud_topic_,
                                   "/input/pointcloud");
    private_nh_.param<std::string>("input_odom_topic", input_odom_topic_,
                                   "/input/odometry");
    private_nh_.param<std::string>("output_cloud_topic", output_cloud_topic_,
                                   "/semantic_pcl/semantic_pcl");
    private_nh_.param<std::string>("world_frame_id", world_frame_id_, "world");
    private_nh_.param<std::string>("sensor_frame_id", sensor_frame_id_, "semantic_sensor");
    private_nh_.param<bool>("publish_sensor_tf", publish_sensor_tf_, true);
    private_nh_.param<int>("sync_queue_size", sync_queue_size_, 10);
    private_nh_.param<double>("sync_slop", sync_slop_, 0.05);

    rgb_bgr_ = loadColorParam("rgb_bgr", {180, 180, 180});
    semantic_bgr_ = loadColorParam("semantic_bgr", {0, 255, 0});

    rgb_value_ = packBgrToFloat(rgb_bgr_);
    semantic_value_ = packBgrToFloat(semantic_bgr_);

    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_, 1);
    cloud_sub_.subscribe(nh_, input_cloud_topic_, 1);
    odom_sub_.subscribe(nh_, input_odom_topic_, 1);

    sync_.reset(new Synchronizer(SyncPolicy(sync_queue_size_), cloud_sub_, odom_sub_));
    sync_->setMaxIntervalDuration(ros::Duration(sync_slop_));
    sync_->registerCallback(
        boost::bind(&PointCloudSemanticBridgeNode::syncedCallback, this, _1, _2));

    ROS_INFO_STREAM("Semantic cloud bridge listening on " << input_cloud_topic_
                                                          << " and " << input_odom_topic_
                                                          << ", publishing " << output_cloud_topic_
                                                          << " with synchronized odometry.");
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, nav_msgs::Odometry>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  /**
   * @brief Describe how optional color-related fields can be reused from an
   * input cloud.
   */
  struct InputFieldLayout
  {
    const sensor_msgs::PointField *packed_rgb_field = nullptr;
    const sensor_msgs::PointField *semantic_color_field = nullptr;
    const sensor_msgs::PointField *r_field = nullptr;
    const sensor_msgs::PointField *g_field = nullptr;
    const sensor_msgs::PointField *b_field = nullptr;

    /**
     * @brief Whether a packed rgb/rgba-style field is available.
     */
    bool hasPackedRgb() const
    {
      return packed_rgb_field != nullptr;
    }

    /**
     * @brief Whether split r/g/b byte channels are available.
     */
    bool hasSplitRgb() const
    {
      return r_field != nullptr && g_field != nullptr && b_field != nullptr;
    }

    /**
     * @brief Whether a semantic_color field is available.
     */
    bool hasSemanticColor() const
    {
      return semantic_color_field != nullptr;
    }
  };

  /**
   * @brief Describe the coordinate fields used to decode point positions.
   */
  struct CoordinateFieldLayout
  {
    const sensor_msgs::PointField *x_field = nullptr;
    const sensor_msgs::PointField *y_field = nullptr;
    const sensor_msgs::PointField *z_field = nullptr;

    /**
     * @brief Whether all coordinate fields are available.
     */
    bool isComplete() const
    {
      return x_field != nullptr && y_field != nullptr && z_field != nullptr;
    }
  };

  /**
   * @brief Pack a BGR triplet into the float representation used by PCL
   * color fields.
   *
   * @param bgr Three-element BGR color vector with values in [0, 255].
   * @return Packed float value suitable for `rgb`-style fields.
   */
  static float packBgrToFloat(const std::vector<int> &bgr)
  {
    uint8_t bytes[4] = {static_cast<uint8_t>(bgr[0]), static_cast<uint8_t>(bgr[1]),
                        static_cast<uint8_t>(bgr[2]), 0};
    float value = 0.0f;
    std::memcpy(&value, bytes, sizeof(float));
    return value;
  }

  /**
   * @brief Pack separate RGB byte channels into the float representation used
   * by PCL color fields.
   *
   * @param red Red channel.
   * @param green Green channel.
   * @param blue Blue channel.
   * @return Packed float value suitable for `rgb`-style fields.
   */
  static float packRgbToFloat(uint8_t red, uint8_t green, uint8_t blue)
  {
    uint8_t bytes[4] = {blue, green, red, 0};
    float value = 0.0f;
    std::memcpy(&value, bytes, sizeof(float));
    return value;
  }

  /**
   * @brief Look up a named field in a PointCloud2 message.
   *
   * @param cloud Input point cloud.
   * @param name Field name to search for.
   * @return Pointer to the matching field or nullptr when missing.
   */
  static const sensor_msgs::PointField *findField(const sensor_msgs::PointCloud2 &cloud,
                                                  const std::string &name)
  {
    for (const sensor_msgs::PointField &field : cloud.fields)
    {
      if (field.name == name)
      {
        return &field;
      }
    }
    return nullptr;
  }

  /**
   * @brief Check whether a field stores a packed 4-byte color value.
   *
   * Supported layouts match the common PointCloud2 encodings used for packed
   * rgb-like fields.
   *
   * @param field Field metadata to validate.
   * @return true When the field can be copied as a packed color value.
   * @return false Otherwise.
   */
  static bool isPackedColorField(const sensor_msgs::PointField &field)
  {
    return field.count == 1 &&
           (field.datatype == sensor_msgs::PointField::FLOAT32 ||
            field.datatype == sensor_msgs::PointField::UINT32 ||
            field.datatype == sensor_msgs::PointField::INT32);
  }

  /**
   * @brief Check whether a field stores a single 8-bit channel value.
   *
   * @param field Field metadata to validate.
   * @return true When the field can be copied as a byte channel.
   * @return false Otherwise.
   */
  static bool isByteChannelField(const sensor_msgs::PointField &field)
  {
    return field.count == 1 &&
           (field.datatype == sensor_msgs::PointField::UINT8 ||
            field.datatype == sensor_msgs::PointField::INT8);
  }

  /**
   * @brief Check whether a field stores coordinates in a format supported by
   * the current decoder.
   *
   * The bridge currently decodes positions through PointCloud2 float
   * iterators, so x/y/z must use scalar FLOAT32 fields.
   *
   * @param field Field metadata to validate.
   * @return true When the field matches the supported coordinate encoding.
   * @return false Otherwise.
   */
  static bool isSupportedCoordinateField(const sensor_msgs::PointField &field)
  {
    return field.count == 1 &&
           field.datatype == sensor_msgs::PointField::FLOAT32;
  }

  /**
   * @brief Convert a PointField datatype enum into a readable name.
   *
   * @param datatype PointField datatype enum value.
   * @return Human-readable datatype name.
   */
  static const char *pointFieldDatatypeName(uint8_t datatype)
  {
    switch (datatype)
    {
    case sensor_msgs::PointField::INT8:
      return "INT8";
    case sensor_msgs::PointField::UINT8:
      return "UINT8";
    case sensor_msgs::PointField::INT16:
      return "INT16";
    case sensor_msgs::PointField::UINT16:
      return "UINT16";
    case sensor_msgs::PointField::INT32:
      return "INT32";
    case sensor_msgs::PointField::UINT32:
      return "UINT32";
    case sensor_msgs::PointField::FLOAT32:
      return "FLOAT32";
    case sensor_msgs::PointField::FLOAT64:
      return "FLOAT64";
    default:
      return "UNKNOWN";
    }
  }

  /**
   * @brief Read a packed 4-byte color value from a specific point field.
   *
   * @param cloud Input cloud storage.
   * @param point_index Point index inside the cloud.
   * @param field Field metadata describing the packed value.
   * @param value Output packed float value.
   * @return true When the value was read successfully.
   * @return false When the field is invalid or out of bounds.
   */
  static bool readPackedFieldValue(const sensor_msgs::PointCloud2 &cloud,
                                   std::size_t point_index,
                                   const sensor_msgs::PointField *field,
                                   float *value)
  {
    if (field == nullptr || !isPackedColorField(*field))
    {
      return false;
    }

    const std::size_t offset = point_index * cloud.point_step + field->offset;
    if (offset + sizeof(float) > cloud.data.size())
    {
      return false;
    }

    std::memcpy(value, &cloud.data[offset], sizeof(float));
    return true;
  }

  /**
   * @brief Read an 8-bit channel value from a specific point field.
   *
   * @param cloud Input cloud storage.
   * @param point_index Point index inside the cloud.
   * @param field Field metadata describing the byte channel.
   * @param value Output channel value.
   * @return true When the channel was read successfully.
   * @return false When the field is invalid or out of bounds.
   */
  static bool readByteChannelValue(const sensor_msgs::PointCloud2 &cloud,
                                   std::size_t point_index,
                                   const sensor_msgs::PointField *field,
                                   uint8_t *value)
  {
    if (field == nullptr || !isByteChannelField(*field))
    {
      return false;
    }

    const std::size_t offset = point_index * cloud.point_step + field->offset;
    if (offset + sizeof(uint8_t) > cloud.data.size())
    {
      return false;
    }

    std::memcpy(value, &cloud.data[offset], sizeof(uint8_t));
    return true;
  }

  /**
   * @brief Resolve which optional input fields can be reused from the source
   * cloud.
   *
   * @param input_cloud Input point cloud from the upstream source.
   * @return Available input field layout for rgb and semantic colors.
   */
  static InputFieldLayout resolveInputFieldLayout(const sensor_msgs::PointCloud2 &input_cloud)
  {
    InputFieldLayout layout;

    const sensor_msgs::PointField *rgb_field = findField(input_cloud, "rgb");
    if (rgb_field != nullptr && isPackedColorField(*rgb_field))
    {
      layout.packed_rgb_field = rgb_field;
    }
    else
    {
      const sensor_msgs::PointField *rgba_field = findField(input_cloud, "rgba");
      if (rgba_field != nullptr && isPackedColorField(*rgba_field))
      {
        layout.packed_rgb_field = rgba_field;
      }
    }

    const sensor_msgs::PointField *semantic_field = findField(input_cloud, "semantic_color");
    if (semantic_field != nullptr && isPackedColorField(*semantic_field))
    {
      layout.semantic_color_field = semantic_field;
    }

    const sensor_msgs::PointField *r_field = findField(input_cloud, "r");
    const sensor_msgs::PointField *g_field = findField(input_cloud, "g");
    const sensor_msgs::PointField *b_field = findField(input_cloud, "b");
    if (r_field != nullptr && g_field != nullptr && b_field != nullptr &&
        isByteChannelField(*r_field) && isByteChannelField(*g_field) &&
        isByteChannelField(*b_field))
    {
      layout.r_field = r_field;
      layout.g_field = g_field;
      layout.b_field = b_field;
    }

    return layout;
  }

  /**
   * @brief Resolve the coordinate field layout from the input cloud.
   *
   * @param input_cloud Input point cloud from the upstream source.
   * @return Available coordinate field layout for x/y/z.
   */
  static CoordinateFieldLayout resolveCoordinateFieldLayout(const sensor_msgs::PointCloud2 &input_cloud)
  {
    CoordinateFieldLayout layout;
    layout.x_field = findField(input_cloud, "x");
    layout.y_field = findField(input_cloud, "y");
    layout.z_field = findField(input_cloud, "z");
    return layout;
  }

  /**
   * @brief Check whether the incoming cloud message is structurally empty.
   *
   * Some upstream nodes publish a default-constructed PointCloud2 while their
   * internal map output is not yet ready. Such messages contain no fields and
   * no data and should be skipped quietly instead of treated as malformed xyz
   * clouds.
   *
   * @param input_cloud Input point cloud from the upstream source.
   * @return true When the cloud contains no schema and no point payload.
   * @return false Otherwise.
   */
  static bool isStructurallyEmptyCloud(const sensor_msgs::PointCloud2 &input_cloud)
  {
    return input_cloud.fields.empty() && input_cloud.data.empty() &&
           input_cloud.width == 0 && input_cloud.height == 0;
  }

  /**
   * @brief Validate that the input cloud uses a supported x/y/z encoding.
   *
   * This avoids silently mis-decoding coordinates when an upstream source
   * publishes non-standard position field types.
   *
   * @param layout Resolved coordinate field layout.
   * @return true When x/y/z can be safely decoded.
   * @return false Otherwise.
   */
  bool validateCoordinateFieldLayout(const CoordinateFieldLayout &layout)
  {
    if (!layout.isComplete())
    {
      if (warned_missing_xyz_topics_.insert(input_cloud_topic_).second)
      {
        ROS_ERROR_STREAM("Input cloud topic " << input_cloud_topic_
                                              << " is missing one of the required x/y/z fields.");
      }
      return false;
    }

    if (!isSupportedCoordinateField(*layout.x_field) ||
        !isSupportedCoordinateField(*layout.y_field) ||
        !isSupportedCoordinateField(*layout.z_field))
    {
      if (warned_unsupported_xyz_topics_.insert(input_cloud_topic_).second)
      {
        ROS_ERROR_STREAM("Input cloud topic " << input_cloud_topic_
                                              << " uses unsupported coordinate field types. "
                                              << "Expected FLOAT32 x/y/z but got x="
                                              << pointFieldDatatypeName(layout.x_field->datatype)
                                              << ", y=" << pointFieldDatatypeName(layout.y_field->datatype)
                                              << ", z=" << pointFieldDatatypeName(layout.z_field->datatype)
                                              << ".");
      }
      return false;
    }

    return true;
  }

  /**
   * @brief Read the rgb value for a point, reusing the input when possible.
   *
   * @param input_cloud Input point cloud from the upstream source.
   * @param point_index Point index inside the cloud.
   * @param layout Resolved input field layout.
   * @param rgb_value Output packed rgb value.
   * @return true When a reusable input rgb value exists.
   * @return false When the caller should fall back to the default rgb value.
   */
  static bool readInputRgbValue(const sensor_msgs::PointCloud2 &input_cloud,
                                std::size_t point_index,
                                const InputFieldLayout &layout,
                                float *rgb_value)
  {
    if (readPackedFieldValue(input_cloud, point_index, layout.packed_rgb_field, rgb_value))
    {
      return true;
    }

    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    if (readByteChannelValue(input_cloud, point_index, layout.r_field, &red) &&
        readByteChannelValue(input_cloud, point_index, layout.g_field, &green) &&
        readByteChannelValue(input_cloud, point_index, layout.b_field, &blue))
    {
      *rgb_value = packRgbToFloat(red, green, blue);
      return true;
    }

    return false;
  }

  /**
   * @brief Read the semantic color for a point, reusing the input when
   * possible.
   *
   * @param input_cloud Input point cloud from the upstream source.
   * @param point_index Point index inside the cloud.
   * @param layout Resolved input field layout.
   * @param semantic_value Output packed semantic color.
   * @return true When a reusable semantic_color field exists.
   * @return false When the caller should fall back to the default semantic
   * color.
   */
  static bool readInputSemanticValue(const sensor_msgs::PointCloud2 &input_cloud,
                                     std::size_t point_index,
                                     const InputFieldLayout &layout,
                                     float *semantic_value)
  {
    return readPackedFieldValue(input_cloud, point_index, layout.semantic_color_field, semantic_value);
  }

  /**
   * @brief Log once how the bridge will populate rgb and semantic fields for
   * the current input topic.
   *
   * @param layout Resolved input field layout.
   */
  void logInputFieldMode(const InputFieldLayout &layout)
  {
    if (!logged_field_mode_topics_.insert(input_cloud_topic_).second)
    {
      return;
    }

    const std::string rgb_mode = layout.hasPackedRgb() || layout.hasSplitRgb()
                                     ? "reusing input rgb fields"
                                     : "using default rgb_bgr";
    const std::string semantic_mode = layout.hasSemanticColor()
                                          ? "reusing input semantic_color field"
                                          : "using default semantic_bgr";
    ROS_INFO_STREAM("Input cloud topic " << input_cloud_topic_ << ": " << rgb_mode
                                         << ", " << semantic_mode << ".");
  }

  /**
   * @brief Load and validate a BGR color parameter from the private
   * namespace.
   *
   * If the parameter is missing, has the wrong size, or contains values
   * outside [0, 255], the provided fallback is returned.
   *
   * @param name Parameter name.
   * @param fallback Default color triplet.
   * @return Validated BGR color triplet.
   */
  std::vector<int> loadColorParam(const std::string &name, const std::vector<int> &fallback)
  {
    std::vector<int> color;
    if (!private_nh_.getParam(name, color) || color.size() != 3)
    {
      ROS_WARN_STREAM("Using default " << name << " because the parameter is missing or invalid.");
      return fallback;
    }

    for (std::size_t i = 0; i < color.size(); ++i)
    {
      if (color[i] < 0 || color[i] > 255)
      {
        ROS_WARN_STREAM("Using default " << name << " because the parameter contains values outside [0, 255].");
        return fallback;
      }
    }
    return color;
  }

  /**
   * @brief Broadcast the synchronized sensor pose derived from odometry as a TF
   * transform.
   *
   * @param odom_msg Odometry message aligned with the input cloud.
   * @param stamp Timestamp used for the transform.
   */
  void publishSensorTf(const nav_msgs::OdometryConstPtr &odom_msg,
                       const ros::Time &stamp)
  {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = world_frame_id_;
    transform.child_frame_id = sensor_frame_id_;
    transform.transform.translation.x = odom_msg->pose.pose.position.x;
    transform.transform.translation.y = odom_msg->pose.pose.position.y;
    transform.transform.translation.z = odom_msg->pose.pose.position.z;
    transform.transform.rotation = odom_msg->pose.pose.orientation;
    tf_broadcaster_.sendTransform(transform);
  }

  /**
   * @brief Decide which frame id should be written to the semantic cloud.
   *
   * If the input cloud is already in the world frame, preserve it to avoid
   * applying the odometry transform twice inside the octomap node.
   *
   * @param input_cloud Input point cloud from the upstream source.
   * @return Frame id used for the output semantic cloud.
   */
  std::string selectOutputFrameId(const sensor_msgs::PointCloud2ConstPtr &input_cloud)
  {
    if (input_cloud->header.frame_id == world_frame_id_)
    {
      if (warned_world_frame_topics_.insert(input_cloud_topic_).second)
      {
        ROS_WARN_STREAM("Input cloud topic " << input_cloud_topic_
                                             << " is already in world frame " << world_frame_id_
                                             << ". Preserving world frame to avoid double transformation.");
      }
      return world_frame_id_;
    }

    return sensor_frame_id_;
  }

  /**
   * @brief Convert a raw point cloud into the semantic point cloud layout
   * consumed by semantic_octomap.
   *
   * Invalid points are discarded. Each valid point is assigned fixed RGB
   * and semantic color values loaded from ROS parameters.
   *
   * @param input_cloud Input point cloud from the simulator.
   * @param output_cloud Converted semantic point cloud message.
   * @return true If at least one valid point was converted.
   * @return false If the cloud is empty, invalid, or missing required fields.
   */
  bool buildSemanticCloud(const sensor_msgs::PointCloud2ConstPtr &input_cloud,
                          sensor_msgs::PointCloud2 *output_cloud)
  {
    if (isStructurallyEmptyCloud(*input_cloud))
    {
      ROS_WARN_THROTTLE(5.0, "Received structurally empty cloud on %s; skipping.", input_cloud_topic_.c_str());
      return false;
    }

    pcl::PointCloud<PointXYZRGBSemantic> semantic_cloud;
    const std::string output_frame_id = selectOutputFrameId(input_cloud);
    const CoordinateFieldLayout coordinate_layout = resolveCoordinateFieldLayout(*input_cloud);
    const InputFieldLayout layout = resolveInputFieldLayout(*input_cloud);
    logInputFieldMode(layout);
    if (!validateCoordinateFieldLayout(coordinate_layout))
    {
      return false;
    }
    semantic_cloud.reserve(static_cast<std::size_t>(input_cloud->width) *
                           static_cast<std::size_t>(input_cloud->height));

    try
    {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*input_cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*input_cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*input_cloud, "z");
      std::size_t point_index = 0;

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++point_index)
      {
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z))
        {
          continue;
        }

        PointXYZRGBSemantic point;
        point.x = *iter_x;
        point.y = *iter_y;
        point.z = *iter_z;
        point.rgb = rgb_value_;
        point.semantic_color = semantic_value_;

        float input_rgb_value = 0.0f;
        if (readInputRgbValue(*input_cloud, point_index, layout, &input_rgb_value))
        {
          point.rgb = input_rgb_value;
        }

        float input_semantic_value = 0.0f;
        if (readInputSemanticValue(*input_cloud, point_index, layout, &input_semantic_value))
        {
          point.semantic_color = input_semantic_value;
        }

        semantic_cloud.push_back(point);
      }
    }
    catch (const std::runtime_error &ex)
    {
      ROS_ERROR_STREAM_THROTTLE(5.0, "Failed to read x/y/z from " << input_cloud_topic_
                                                                  << ": " << ex.what());
      return false;
    }

    if (semantic_cloud.empty())
    {
      ROS_WARN_THROTTLE(5.0, "Received empty or invalid cloud on %s", input_cloud_topic_.c_str());
      return false;
    }

    semantic_cloud.width = semantic_cloud.size();
    semantic_cloud.height = 1;
    semantic_cloud.is_dense = true;

    pcl::toROSMsg(semantic_cloud, *output_cloud);
    output_cloud->header.stamp = input_cloud->header.stamp;
    output_cloud->header.frame_id = output_frame_id;
    return true;
  }

  /**
   * @brief Process synchronized cloud and odometry messages.
   *
   * The callback first updates the TF tree with the pose extracted from the
   * current odometry message and then republishes the input cloud in semantic
   * point cloud format.
   *
   * @param cloud_msg Input point cloud from the upstream source.
   * @param odom_msg Synchronized odometry for the same measurement.
   */
  void syncedCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                      const nav_msgs::OdometryConstPtr &odom_msg)
  {
    if (publish_sensor_tf_)
    {
      publishSensorTf(odom_msg, cloud_msg->header.stamp);
    }

    sensor_msgs::PointCloud2 semantic_cloud;
    if (!buildSemanticCloud(cloud_msg, &semantic_cloud))
    {
      return;
    }

    cloud_pub_.publish(semantic_cloud);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher cloud_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  std::unique_ptr<Synchronizer> sync_;

  std::string input_cloud_topic_;
  std::string input_odom_topic_;
  std::string output_cloud_topic_;
  std::string world_frame_id_;
  std::string sensor_frame_id_;
  bool publish_sensor_tf_;
  int sync_queue_size_;
  double sync_slop_;
  std::vector<int> rgb_bgr_;
  std::vector<int> semantic_bgr_;
  float rgb_value_;
  float semantic_value_;
  std::unordered_set<std::string> logged_field_mode_topics_;
  std::unordered_set<std::string> warned_missing_xyz_topics_;
  std::unordered_set<std::string> warned_unsupported_xyz_topics_;
  std::unordered_set<std::string> warned_world_frame_topics_;
};

/**
 * @brief Entry point for the semantic point cloud bridge node.
 *
 * @param argc Number of command line arguments.
 * @param argv Command line arguments.
 * @return int Process exit code.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_semantic_cloud_bridge");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  PointCloudSemanticBridgeNode node(nh, private_nh);
  ros::spin();
  return 0;
}
