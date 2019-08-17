include "00_trajectory_builder_2d.lua" -- local 2D SLAM Einstellungen
include "00_trajectory_builder_3d.lua" -- local 3D SLAM Einstellungen
include "00_pose_graph.lua" -- gobal SLAM Einstellungen

-- Festlegen ob 2D oder 3D-SLAM verwentet wird
MAP_BUILDER = {
  use_trajectory_builder_2d = true,
  use_trajectory_builder_3d = false,
  num_background_threads = 4,	-- kann bis auf die anzahl Cores des Computers erhöt werden.
  pose_graph = POSE_GRAPH, -- zuweisung der Einstellungen
}

-- Hier werden die local SLAM Einstellungen zugewiesen
TRAJECTORY_BUILDER = {
  trajectory_builder_2d = TRAJECTORY_BUILDER_2D,
  trajectory_builder_3d = TRAJECTORY_BUILDER_3D,
  pure_localization = false, -- begrenzt die Anzahl Submaps auf 3 und löscht die älteren, um Rechenleistung zu sparen
}

-- Eigentliche Einstellugnen die dem Cartographer übergeben werden. 
options = {
-- zuweisen der oben zusammengefassten Einstellungen
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
-- frame zuweisungen
  map_frame = "map",	-- fixed Frame
  tracking_frame = "base_footprint", -- wird am besten zu "imu_link" gesetzt, da das IMU-Frame keinen versatzt zum tracking_frame haben darf. 
  published_frame = "base_footprint", -- ist das child_frame für die tf-Messages des Cartographers, also der Roboter base_link 
  odom_frame = "odom",	-- nötig für provide_odom_frame
  provide_odom_frame = true, 	-- gibd non-loopclosed Position des local SLAM auf dem oben festgelegten Topic. Wird verwendet wenn keine tf
--Messages von der Roboter Odometrie versendet werden
  publish_frame_projected_to_2d = true, -- durch die pose extrapolation kann es im 2D-SLAM zu out-off-plane Positionen kommen. 
--So ist sie auf zwei Dimensionen eingeschränkt
-- input data  
  use_odometry = true, -- Odometrie
  use_nav_sat = false, -- GPS
  use_landmarks = false,
  num_laser_scans = 1, -- Anzal laser_scan Topics (2D LiDAR, sensor_msgs/LaserScan)
  num_multi_echo_laser_scans = 0, -- Anzahl multi_echo_laser_scans (multi echo LiDAR, sensor_msgs/MultiEchoLaserScan)
  num_subdivisions_per_laser_scan = 1, -- Anzahl der Pointclouds pro multi_echo_laser_scan Topic
  num_point_clouds = 0, -- num point_cloud topics (3D LiDAR, sensor_msgs/PointCloud2)
-- publishing-rate für die verschiedenen Topics des Cartographers (wurde nicht verändert)
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
-- Sampelt die Sensordaten im gegebenen verhältniss (wurde nicht verändert)
  rangefinder_sampling_ratio = 1,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}


return options
