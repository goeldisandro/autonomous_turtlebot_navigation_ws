######################################################################### online sehr langsam aber top resultat


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

  

   

  

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  

   

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true  

--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 100  

--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 4  

  

   

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10  

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 0.1  

  

   

--TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5  

--TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 50  

TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1) 

  

   

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 75  

  

   

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

TRAJECTORY_BUILDER_2D.use_imu_data = true  

TRAJECTORY_BUILDER_2D.min_range = 1  

  

   

MAP_BUILDER.use_trajectory_builder_2d = true  

MAP_BUILDER.num_background_threads = 7  

   

POSE_GRAPH.optimization_problem.huber_scale = 5e2  

POSE_GRAPH.optimize_every_n_nodes = 2  

POSE_GRAPH.constraint_builder.sampling_ratio = 0.03  

POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10  

POSE_GRAPH.constraint_builder.min_score = 0.62  

POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66  

  

   

  

POSE_GRAPH.optimization_problem.odometry_rotation_weight = 10  

  

   

  

return options  


###################################################### online lua mit optimalem Resultat aber langsam


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

  

   

  

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  

   

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true  

--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 100  

--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 4  

  

   

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10  

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 0.1  

  

   

--TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5  

--TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 50  

TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1) 

  

   

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 75  

  

   

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

TRAJECTORY_BUILDER_2D.use_imu_data = true  

TRAJECTORY_BUILDER_2D.min_range = 1  

  

   

MAP_BUILDER.use_trajectory_builder_2d = true  

MAP_BUILDER.num_background_threads = 7  

   

POSE_GRAPH.optimization_problem.huber_scale = 5e2  

POSE_GRAPH.optimize_every_n_nodes = 2  

POSE_GRAPH.constraint_builder.sampling_ratio = 0.03  

POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10  

POSE_GRAPH.constraint_builder.min_score = 0.62  

POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66  

  

   

  

POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1  

  

   

  

return options  

###################################################### lua nach Bosser ohne online scan matching
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

  num_laser_scans = 0,  

  num_multi_echo_laser_scans = 0,  

  num_subdivisions_per_laser_scan = 1,  

  num_point_clouds =1,  

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

  

   

  

--TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  

   

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true  

--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 100  

--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 4  

  

   

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10  

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 0.1  

  

   

--TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5  

--TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 50  

TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1) 

  

   

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 75  

  

   

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 

--TRAJECTORY_BUILDER_2D.use_imu_data = false  

TRAJECTORY_BUILDER_2D.min_range = 1  

  

   

MAP_BUILDER.use_trajectory_builder_2d = true  

MAP_BUILDER.num_background_threads = 7  

   

POSE_GRAPH.optimization_problem.huber_scale = 5e2  

POSE_GRAPH.optimize_every_n_nodes = 2  

POSE_GRAPH.constraint_builder.sampling_ratio = 0.03  

POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10  

POSE_GRAPH.constraint_builder.min_score = 0.62  

POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66  

  

   

  

POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1  

  

   

  

return options  






################################################ lua der ersten online matching

<launch>

<param name="/use_sim_time" value="false"/>

<!-- IMU starten ####################################################################-->


<!--Velodyne starten #################################################################### -->
  <!-- declare arguments with default values -->
  <arg name="calibration" default="$(find velodyne_pointcloud)/params/VLP16db.yaml"/>
  <arg name="device_ip" default="" />
  <arg name="frame_id" default="velodyne" />
  <arg name="manager" default="$(arg frame_id)_nodelet_manager" />
  <arg name="max_range" default="130.0" />
  <arg name="min_range" default="0.4" />
  <arg name="pcap" default="" />
  <arg name="port" default="2368" />
  <arg name="read_fast" default="false" />
  <arg name="read_once" default="false" />
  <arg name="repeat_delay" default="0.0" />
  <arg name="rpm" default="600.0" />
  <arg name="gps_time" default="false" />
  <arg name="cut_angle" default="-0.01" />
  <arg name="laserscan_ring" default="-1" />
  <arg name="laserscan_resolution" default="0.007" />

  <!-- start nodelet manager and driver nodelets -->
  <include file="$(find velodyne_driver)/launch/nodelet_manager.launch">
    <arg name="device_ip" value="$(arg device_ip)"/>
    <arg name="frame_id" value="$(arg frame_id)"/>
    <arg name="manager" value="$(arg manager)" />
    <arg name="model" value="VLP16"/>
    <arg name="pcap" value="$(arg pcap)"/>
    <arg name="port" value="$(arg port)"/>
    <arg name="read_fast" value="$(arg read_fast)"/>
    <arg name="read_once" value="$(arg read_once)"/>
    <arg name="repeat_delay" value="$(arg repeat_delay)"/>
    <arg name="rpm" value="$(arg rpm)"/>
    <arg name="gps_time" value="$(arg gps_time)"/>
    <arg name="cut_angle" value="$(arg cut_angle)"/>
  </include>

  <!-- start cloud nodelet -->
  <include file="$(find velodyne_pointcloud)/launch/cloud_nodelet.launch">
    <arg name="calibration" value="$(arg calibration)"/>
    <arg name="manager" value="$(arg manager)" />
    <arg name="max_range" value="$(arg max_range)"/>
    <arg name="min_range" value="$(arg min_range)"/>
  </include>

  <!-- start laserscan nodelet -->
  <include file="$(find velodyne_pointcloud)/launch/laserscan_nodelet.launch">
    <arg name="manager" value="$(arg manager)" />
    <arg name="ring" value="$(arg laserscan_ring)"/>
    <arg name="resolution" value="$(arg laserscan_resolution)"/>
  </include>  




<!-- RVIZ starten ####################################################################-->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find rplidar_ros)/rviz/rplidar.rviz" />




<!-- eigene tf_broadcaster/listener einbinden############################################## -->
<node name="tf_broadcaster" pkg="robot_setup_tf" type="tf_broadcaster" /> 
<node name="tf_broadcaster_base" pkg="robot_setup_tf" type="tf_broadcaster_base" /> 



<!-- Turtlebot  bringup ################################################################## -->
<include file =  "$(find turtlebot_bringup)/launch/minimal.launch"/>


<!---Cartographer ######################3     
..._without_imu = ohne IMU -->
  <param name="robot_description"
    textfile="$(find cartographer_ros)/urdf/00_Kobuki_v_laserscan.urdf" />
  <node name="cartographer_node" pkg="cartographer_ros"
      type="cartographer_node" args="
          -configuration_directory $(find cartographer_ros)/configuration_files
          -configuration_basename 00_Kobuki_v_laserscan.lua
          -load_state_filename $(arg load_state_filename)"
      output="screen">
      <remap from="points2" to="velodyne_points" />
  </node>

  <node name="cartographer_occupancy_grid_node" pkg="cartographer_ros"
      type="cartographer_occupancy_grid_node" args="-resolution 0.05" />


<!-- move base node  ########################################################################## 
local_costmap params macht keine Probleme mehr -> Kommentare aus .yaml file geloescht-->
  <param name="robot_description"
    textfile="$(find cartographer_ros)/urdf/00_Kobuki_v_laserscan.urdf" />

   <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
   <rosparam file="$(find kobuki_nav)/costmap_common_params.yaml" command="load" ns="global_costmap" /> 
   <rosparam file="$(find kobuki_nav)/costmap_common_params.yaml" command="load" ns="local_costmap" />
   <rosparam file = "$(find kobuki_nav)/local_costmap_params.yaml" command="load" />
   <rosparam file="$(find kobuki_nav)/global_costmap_params.yaml" command="load" /> 
   <rosparam file="$(find kobuki_nav)/base_local_planner_params.yaml" command="load" /> 
   </node>
  <!---->

    <!-- run laserscan_to_pointcloud node -->
  <arg name="use_nodelet" default="true"/>
  <arg name="nodelet_manager" default="velodyne_nodelet_manager"/>

  <arg name="cloud_in" default="/velodyne_points"/>
  <arg name="scan_out" default="/scan_navigation"/>

  <arg name="min_height" default="-0.2"/>
  <arg name="max_height" default="0.5"/>
  <arg name="scan_time" default="0.05"/>
  <arg name="range_max" default="10"/>
  <arg name="angle_max" default="3.14159"/>
  <arg name="angle_min" default="-3.14159"/>

  <node if="$(arg use_nodelet)" name="pointcloud_to_navigation_laserscan" pkg="nodelet" type="nodelet"
        args="load pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet $(arg nodelet_manager)">
    <remap from="cloud_in" to="$(arg cloud_in)"/>
    <remap from="scan" to="$(arg scan_out)"/>
    <param name="min_height" value="$(arg min_height)"/>
    <param name="max_height" value="$(arg max_height)"/>
    <param name="scan_time" value="$(arg scan_time)"/>
    <param name="range_max" value="$(arg range_max)"/>
    <param name="angle_max" value="$(arg angle_max)"/>
    <param name="angle_min" value="$(arg angle_min)"/>
  </node>

  <node unless="$(arg use_nodelet)" name="pointcloud_to_navigation_laserscan" pkg="pointcloud_to_laserscan" type="pointcloud_to_laserscan_node">
    <remap from="cloud_in" to="$(arg cloud_in)"/>
    <remap from="scan" to="$(arg scan_out)"/>
    <param name="min_height" value="$(arg min_height)"/>
    <param name="max_height" value="$(arg max_height)"/>
    <param name="scan_time" value="$(arg scan_time)"/>
    <param name="range_max" value="$(arg range_max)"/>
    <param name="angle_max" value="$(arg angle_max)"/>
    <param name="angle_min" value="$(arg angle_min)"/>
  </node>
</launch>







################################################################################################################################################################
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map", 
  tracking_frame = "base_link", 
  published_frame = "base_link", 
  odom_frame = "odom", 
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 16,--1
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

--TRAJECTORY_BUILDER_2D.min_range = 0.1
--TRAJECTORY_BUILDER_2D.max_range = 8.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 75
--TRAJECTORY_BUILDER_2D.min_z = 0.2 

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7





--TRAJECTORY_BUILDER.pure_localization = true
--TRAJECTORY_BUILDER.pure_localization_trimmer = {
--  max_submaps_to_keep = 3,
--}
--POSE_GRAPH.optimize_every_n_nodes = 20
--options.published_frame = "map"

return options
