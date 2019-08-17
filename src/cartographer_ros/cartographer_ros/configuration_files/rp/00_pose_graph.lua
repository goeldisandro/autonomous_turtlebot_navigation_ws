POSE_GRAPH = {
  optimize_every_n_nodes = 20,		-- set to 0 for testing local slam (global slam is run every n nodes)
  
-- Sucht nach Constraints
  constraint_builder = {
    fast_correlative_scan_matcher = {	-- 2D Sucht passensde Maps (Submaps - Nodes) -> Constraint
      linear_search_window = 2.,	-- je kleiner desto schneller
      angular_search_window = math.rad(15.),
      branch_and_bound_depth = 7,
    },
    ceres_scan_matcher = {	-- 2D berechnet die Anpassung (Residual) aufgrund von vielen Einflüssen (gewichtet)
      occupied_space_weight = 20.,
      translation_weight = 1.,
      rotation_weight = 1.,
      ceres_solver_options = {
        use_nonmonotonic_steps = true,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
    fast_correlative_scan_matcher_3d = { -- 2D Sucht passensde Maps (Submaps - Nodes) -> Constraint
      branch_and_bound_depth = 8, -- begrenzd den Suchmechanismus bis zum resultat (branch and bound)
      full_resolution_depth = 3, --Number of full resolution grids to use, additional grids will reduce the resolution by halfeach
      min_rotational_score = 0.77,
      min_low_resolution_score = 0.55,
      linear_xy_search_window = 2.,
      linear_z_search_window = 1.,
      angular_search_window = math.rad(20.),
    },
    ceres_scan_matcher_3d = {	-- Einstellungen 3d
      occupied_space_weight_0 = 20.,
      occupied_space_weight_1 = 30.,
      translation_weight = 10.,
      rotation_weight = 1.,
      only_optimize_yaw = false,
      ceres_solver_options = {
        use_nonmonotonic_steps = false,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
    sampling_ratio = 0.05, --A constraint will be added if the proportion of added constraints to potential constraints dropsbelow this number
    max_constraint_distance = 15., --max Abstand von einem Node zu einer Submap
    min_score = 0.55,	-- min Score des FastCorrelativScanMatchers
    global_localization_min_score = 0.6,
    loop_closure_translation_weight = 1.1e4,  	--1.1e4
    loop_closure_rotation_weight = 1e5,		--1e5
    log_matches = true,
  },

-- optimiert den Pose Graph anhand aller constraints
  matcher_translation_weight = 5e2,--5e2   of the non-global (matcher) constraints
  matcher_rotation_weight = 1.6e3,--1.6e3	 of the non-global (matcher) constraints	
  optimization_problem = {
    huber_scale = 1e1,
    acceleration_weight = 1,--1e3		  of the IMU data
    rotation_weight = 3,--3e5
    local_slam_pose_translation_weight = 1e5,	--of the local slam pose
    local_slam_pose_rotation_weight = 1e5,
    odometry_translation_weight = 100,		--of the odometry
    odometry_rotation_weight = 15,
    fixed_frame_pose_translation_weight = 1e1,	--of the GPS
    fixed_frame_pose_rotation_weight = 1e2,
    log_solver_summary = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 50,
      num_threads = 7,
    },
  },
  max_num_final_iterations = 150,
  global_sampling_ratio = 0.0005, -- Rate at which we sample a single trajectory’s nodes for global localization.
  log_residual_histograms = true,
  global_constraint_search_after_n_seconds = 10.,
}
