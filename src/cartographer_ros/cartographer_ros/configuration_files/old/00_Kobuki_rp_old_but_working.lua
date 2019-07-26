include "map_builder.lua" 

include "trajectory_builder.lua" 

  

options = { 

  map_builder = MAP_BUILDER, 

  trajectory_builder = TRAJECTORY_BUILDER, 

  

  map_frame = "map", 

  tracking_frame = "base_footprint", 

  published_frame = "base_footprint", 

  odom_frame = "odom", 
  provide_odom_frame = true,  

  publish_frame_projected_to_2d = false,  

  use_odometry = true,  

  use_nav_sat = false,  

  use_landmarks = false,  

  num_laser_scans = 1,  

  num_multi_echo_laser_scans = 0,  

  num_subdivisions_per_laser_scan = 1,  

  num_point_clouds =0,  

  lookup_transform_timeout_sec = 0.2,  

  submap_publish_period_sec = 0.3,  

  pose_publish_period_sec = 5e-3,  

  trajectory_publish_period_sec = 30e-3,  

  rangefinder_sampling_ratio = 1.,  

  odometry_sampling_ratio = 1,  

  fixed_frame_pose_sampling_ratio = 1.,  

  imu_sampling_ratio = 1.,  

  landmarks_sampling_ratio = 1.,  

  

}  


--###########################################################################
-- optimierte Werte für RP Lidar, Kobuki Odom, IMU via BBB
  
-- Trajectory Builder
TRAJECTORY_BUILDER_2D.use_imu_data=false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true 
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1 --optimiert


TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10 --optimiert
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1000 --optimiert
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.1 -- optimiert
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters =10  --optimiert
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.8 --optimiert
TRAJECTORY_BUILDER_2D.submaps.num_range_data =60 --optimiert
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 --optimiert
TRAJECTORY_BUILDER_2D.min_range = 0.2 --optimiert
TRAJECTORY_BUILDER_2D.max_range =10 --optimiert
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 9.1
--optimiert
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight =0.2 
--optimiert

  

MAP_BUILDER.use_trajectory_builder_2d = true 

MAP_BUILDER.num_background_threads = 6 

  

POSE_GRAPH.optimization_problem.huber_scale = 5e2

POSE_GRAPH.optimize_every_n_nodes = 8

POSE_GRAPH.constraint_builder.sampling_ratio = 0.03 

POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 100 

POSE_GRAPH.constraint_builder.min_score =0.4


POSE_GRAPH.constraint_builder.global_localization_min_score = 0.8 --wie schnell die globale Karte gematcht werden soll

  

POSE_GRAPH.optimization_problem.odometry_rotation_weight = 5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 150
  

return options 
