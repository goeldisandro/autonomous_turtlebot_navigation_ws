MAX_3D_RANGE = 50.

TRAJECTORY_BUILDER_3D = {
  min_range = 1.5, 	 		-- Filtert Punkte näher als der angegebene Wert
  max_range = MAX_3D_RANGE, 		-- Filtert Punkte weiterweg als der angegebene Wert
  num_accumulated_range_data = 80, 	-- wieviele Einzelne PointCloud2 Messages zusammen fürs scan-matching verwendet werden (sollte 360° abdecken)
  
  voxel_filter_size = 0.15, 		-- fixer Filter: Alle Punkte in einem Voxel werden zu einem zum Voxel-Mittelpunkt zusammengefasst

  high_resolution_adaptive_voxel_filter = { -- adaptiver Filter
    max_length = 0.5, -- maximale grösse der Voxels
    min_num_points = 100, -- verkleinert voxels bis min-Anzahl RANGE-Punkte enthalten sind. 
    max_range = 20., -- filtert alle Punkte weiter weg
  },

  low_resolution_adaptive_voxel_filter = {
    max_length = 4.,
    min_num_points = 100,
    max_range = MAX_3D_RANGE,
  },

-- RealTimeCorrelativeScanMacher sucht in einem gewissen Fenstern eine InitalPose um den Scan einzusetzen
-- sehr rechen intensive, aber von vorteil wenn IMU und Odom-Daten schlecht oder nicht vorhanden sind
-- überschreibt diese Sensordaten
  use_online_correlative_scan_matching = false,
  real_time_correlative_scan_matcher = {
    linear_search_window = 2,
    angular_search_window = math.rad(10.),
    translation_delta_cost_weight = 1e-1,
    rotation_delta_cost_weight = 1e-1,
  },

-- Findet Optimale Position auf gund der InitialPose aus Sensordaten oder aus RealTimeCorrelativeScanMacher
  ceres_scan_matcher = {
    occupied_space_weight_0 = 6.,	-- nerly no impact
    occupied_space_weight_1 = 1.,	-- nerly no impact
    translation_weight = 10,
    rotation_weight = 0.1,		-- wackeln der IMU wird mit einbezogen
    only_optimize_yaw = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 12,
      num_threads = 1,
    },
  },

-- Festlegen ob ein Scan eingefügt wird oder nicht
  motion_filter = {
    max_time_seconds = 5, -- nach 5 sec wird immer eingefügt
    max_distance_meters = 0.5,  -- wenn zum letzten Scan ein gewisser Abstand festgestellt wird
    max_angle_radians =  math.rad(5.), -- wenn zum letzten Scan eine gewisse Rotation festgestellt wird
  },

  imu_gravity_time_constant = 10.,	 -- über diese Zeit wird die Gravitationsrichtung gemittelt
  rotational_histogram_size = 120,

-- Einstellungen zum Erstellen der Submaps
  submaps = {
    high_resolution = 0.15,		-- festlegen der Auflösung der beiden Probability-Grids
    high_resolution_max_range = 20., -- max Dinstanz des hochaufgelösten Probability-Grids
    low_resolution = 0.5,		--
    num_range_data = 50,		-- Grösse der Submap, so wählen, dass sie in sich konsistent ist
    range_data_inserter = {		-- Gewichtung von Hits und Misses im Wahrscheinlichkeits-Gitter
      hit_probability = 0.55,		-- 0.55
      miss_probability = 0.49,		-- 0.49
      num_free_space_voxels = 2,	-- 0.1
    },
  },
}
