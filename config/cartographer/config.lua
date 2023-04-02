include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.1,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}


MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 5
TRAJECTORY_BUILDER_2D.use_imu_data = true 
TRAJECTORY_BUILDER_2D.max_range = 24
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20 
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 40
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 60
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.01
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.75
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.47

TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1


POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1E4
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1E4
POSE_GRAPH.optimize_every_n_nodes = 150 
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 0.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(20.)
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 20.
POSE_GRAPH.constraint_builder.max_constraint_distance = 25.
POSE_GRAPH.global_constraint_search_after_n_seconds = 60.

return options
