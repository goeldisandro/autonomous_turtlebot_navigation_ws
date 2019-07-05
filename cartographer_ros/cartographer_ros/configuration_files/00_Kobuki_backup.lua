

##################################### gemäss Word optimiertes Lua
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

  num_laser_scans =0, 

  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1, 

  num_point_clouds = 1, 

  lookup_transform_timeout_sec = 0.2, 

  submap_publish_period_sec = 0.3, 

  pose_publish_period_sec = 5e-3, 

  trajectory_publish_period_sec = 30e-3, 

  rangefinder_sampling_ratio = 1., 

  odometry_sampling_ratio = 1, --1 

  fixed_frame_pose_sampling_ratio = 1., 

  imu_sampling_ratio = 1., 

  landmarks_sampling_ratio = 1., 

} 

  


--###########################################################################
-- optimierte Werte für RP Lidar, Kobuki Odom, IMU via BBB
  
-- Trajectory Builder
TRAJECTORY_BUILDER_2D.use_imu_data=true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true 
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1 --optimiert


TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 500 --optimiert
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1000 --optimiert
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.1 -- optimiert
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters =10  --optimiert
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.8 --optimiert
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35 --optimiert
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10 --optimiert
TRAJECTORY_BUILDER_2D.min_range = 0.5 --optimiert
TRAJECTORY_BUILDER_2D.max_range =10 --optimiert
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.  
--optimiert
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight =0.2 
--optimiert


--Pose Graph
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 10--

--Map Builder
MAP_BUILDER.use_trajectory_builder_2d = true 


return options 







######################################################################
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

  num_point_clouds = 0, 

  lookup_transform_timeout_sec = 0.2, 

  submap_publish_period_sec = 0.3, 

  pose_publish_period_sec = 5e-3, 

  trajectory_publish_period_sec = 30e-3, 

  rangefinder_sampling_ratio = 1., 

  odometry_sampling_ratio = 1, --1 

  fixed_frame_pose_sampling_ratio = 1., 

  imu_sampling_ratio = 1., 

  landmarks_sampling_ratio = 1., 

} 

  

--TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
--########################################################################### Settings for online SLAM

--TRAJECTORY_BUILDER.pure_localization = false
--POSE_GRAPH.optimize_every_n_nodes = 1
--MAP_BUILDER.num_background_threads = 8
--POSE_GRAPH.global_sampling_ratio=0.003
--POSE_GRAPH.constraint_builder.sampling_ratio = 0.3 
--POSE_GRAPH.constraint_builder.min_score = 0.65 

--settings in 3d should have changed: https://github.com/googlecartographer/cartographer_ros/issues/1015
--TRAJECTORY_BUILDER.pure_localization_trimmer = {
  --max_submaps_to_keep = 3,
--}
--POSE_GRAPH.optimize_every_n_nodes = 2

--###########################################################################

  

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true 

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 100 --1 --10 y

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1000 --1 

  

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5 

TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 50 

TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 50 

  

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 50 --50 

  

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 100 --recomended 100 

TRAJECTORY_BUILDER_2D.use_imu_data=false
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1000
MAP_BUILDER.use_trajectory_builder_2d = true 
-- ################################ ab hier kopierte Werte
TRAJECTORY_BUILDER_2D.min_range = 0.1 --1
TRAJECTORY_BUILDER_2D.max_range = 10
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1


return options 





--############################################################################################################################
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

  num_point_clouds = 0, 

  lookup_transform_timeout_sec = 0.2, 

  submap_publish_period_sec = 0.3, 

  pose_publish_period_sec = 5e-3, 

  trajectory_publish_period_sec = 30e-3, 

  rangefinder_sampling_ratio = 1., 

  odometry_sampling_ratio = 1, --1 

  fixed_frame_pose_sampling_ratio = 1., 

  imu_sampling_ratio = 1., 

  landmarks_sampling_ratio = 1., 

} 

  


--###########################################################################
-- optimierte Werte für RP Lidar, Kobuki Odom, MPU 6050 IMU via Arduino
  
-- Trajectory Builder
TRAJECTORY_BUILDER_2D.use_imu_data=true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true 
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.05 --mit vergrösserung dieses Parameters steigt der Rechenaufwand enorm an!


TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 500 --optimiert
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 100 --optimiert 
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1 --optimiert
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 20 --optimiert
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 80 --optimiert
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35 --optimiert
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 15 --optimiert
TRAJECTORY_BUILDER_2D.min_range = 0.0001 --optimiert
TRAJECTORY_BUILDER_2D.max_range =10 --optimiert
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 5.  
--optimiert
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.3 
--optimiert


--Pose Graph
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 250  --optimiert

--Map Builder
MAP_BUILDER.use_trajectory_builder_2d = true 


return options 




