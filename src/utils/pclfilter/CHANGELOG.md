CHANGELOG for pclfilter

2026-03-05 - Improvements for mid_360 and point-lio
- preprocess_node.py:
  - Vectorized ground-min computation using `np.minimum.at` (replaced Python loops).
  - Added `sensor_type` parameter with `mid_360` preset (voxel_size=0.06, ground_grid=0.4, ground_thresh=0.15, max_range=80.0).
  - Added `point_lio_topic` publisher (default `/point_lio/points`) to forward non-ground points to point-lio.
  - Optimized PointCloud2 reading using `np.fromiter` with fallback to existing method.
- launch/preprocess.launch.py:
  - Added `point_lio_topic` and set `sensor_type` default to `mid_360`.

Notes:
- Tune the preset values for `mid_360` on real data: voxel size, grid size and ground threshold.
- Update other launch files if different defaults are required for other sensors.
