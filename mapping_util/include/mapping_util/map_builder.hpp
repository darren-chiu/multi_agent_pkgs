#ifndef MAPPING_UTIL_MAP_BUILDER_CLASS_H_
#define MAPPING_UTIL_MAP_BUILDER_CLASS_H_

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "path_tools.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "voxel_grid.hpp"
#include <env_builder_msgs/msg/voxel_grid_stamped.hpp>
#include <env_builder_msgs/srv/get_voxel_grid.hpp>
#include <iostream>
#include <mutex>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stdlib.h>
#include <string>

namespace mapping_util {
class MapBuilder : public ::rclcpp::Node {
public:
  MapBuilder();

private:
  /*-------------- methods ---------------*/
  // declare ros parameters
  void DeclareRosParameters();

  // initialize ros parameters
  void InitializeRosParameters();

  // callback for when we receive the map from the environment
  void EnvironmentVoxelGridCallback(
      const ::env_builder_msgs::msg::VoxelGridStamped::SharedPtr vg_msg);

  // merge 2 grids together; the unknown voxels of the new grid are replaced
  // with the voxels of the old grid
  ::voxel_grid_util::VoxelGrid
  MergeVoxelGrids(const ::voxel_grid_util::VoxelGrid &vg_old,
                  const ::voxel_grid_util::VoxelGrid &vg_new);

  // raycast from a point inside the voxel grid (local coordinates) to clear all
  // the voxels in the way
  void RaycastAndClear(::voxel_grid_util::VoxelGrid &vg,
                       const ::Eigen::Vector3d &start);

  // go through all the voxels that are free, and set those
  // that have at least one unknown neighbours as also unknown because due
  // to the inflation they can become occupied; if it has one occupied
  // neighbour or all its neighbours are free; it is kept as free
  void SetUncertainToUnknown(::voxel_grid_util::VoxelGrid &vg);

  // clear the line along the start and the end in the voxel
  void ClearLine(::voxel_grid_util::VoxelGrid &vg,
                 ::voxel_grid_util::VoxelGrid &vg_final,
                 const ::Eigen::Vector3d &start, const ::Eigen::Vector3d &end);

  // callback for when we receive the new agent position
  void TfCallback(const ::tf2_msgs::msg::TFMessage::SharedPtr msg);

  // display computation time
  void DisplayCompTime(::std::vector<double> &comp_time);

  // function to execute on the shutdown of the node to save computation time
  // statistics
  void OnShutdown();

  /*-------------- member variables ---------------*/
  /* ROS parameters */
  // environment voxel grid topic name
  ::std::string env_vg_topic_;
  // agent id
  int id_;
  // voxel grid range
  ::std::vector<double> voxel_grid_range_;
  // world frame name
  ::std::string world_frame_;
  // agent frame name
  ::std::string agent_frame_;
  // whether to free all voxels that are not occupied (no unknowns); if not then
  // we have to raycast to free the voxels
  bool free_grid_;
  // inflation distance
  double inflation_dist_;
  // potential distance and power
  double potential_dist_;
  double potential_pow_;

  /* publishers/subscribers */
  // environment voxel grid subscriber
  ::rclcpp::Subscription<::env_builder_msgs::msg::VoxelGridStamped>::SharedPtr
      voxel_grid_sub_;
  // tf buffer
  ::std::shared_ptr<::tf2_ros::Buffer> tf_buffer_;
  // tf listener to get the agent position
  ::std::shared_ptr<::tf2_ros::TransformListener> tf_listener_;
  // tf subscriber
  ::rclcpp::Subscription<::tf2_msgs::msg::TFMessage>::SharedPtr tf_subscriber_;
  // voxel grid publisher
  ::rclcpp::Publisher<::env_builder_msgs::msg::VoxelGridStamped>::SharedPtr
      voxel_grid_pub_;

  // current position of the agent
  ::std::vector<double> pos_curr_;

  // current voxel grid
  ::voxel_grid_util::VoxelGrid voxel_grid_curr_;

  /* mutexes for memory management - unnecessary for now because the callbacks
   * are executed sequentially but in case we use MultiThreadedExecutor in the
   * future */
  // mutex for position update
  ::std::mutex pos_mutex_;

  // raycast computation time
  ::std::vector<double> raycast_comp_time_;
  ::std::vector<double> merge_comp_time_;
  ::std::vector<double> tot_comp_time_;
};

// convert ::env_builder_msgs::msg::VoxelGrid to ::voxel_grid_util::VoxelGrid
::voxel_grid_util::VoxelGrid
ConvertVGMsgToVGUtil(::env_builder_msgs::msg::VoxelGrid &vg_msg);

// convert ::voxel_grid_util::VoxelGrid to ::env_builder_msgs::msg::VoxelGrid
::env_builder_msgs::msg::VoxelGrid
ConvertVGUtilToVGMsg(::voxel_grid_util::VoxelGrid &vg);
} // namespace mapping_util
#endif
