# Sphere-VIO MSCKF Improvement Log

## Baseline

- Prior full-sequence MSCKF on `manual_small_1-sync.bag` used only cross-camera
  stereo `LandmarkTrack` features. It created 162 tracks and diverged:
  `ATE translation RMSE = 5208.22 m`.
- The old offline path required an intermediate converted Sphere-VIO bag and
  reused the current-frame `point_b` with an arbitrary earlier clone pose,
  which is geometrically inconsistent for temporally tracked features.

## Algorithm Changes

1. Direct D2SLAM ingestion: decode `/arducam/image/compressed` once per frame
   and split it into four synchronized mono8 images, avoiding intermediate
   full-size bags. Corrected the per-camera image loop so all four sub-images
   are enqueued before assembling a frame.
2. Raw D2SLAM IMU selection: use `/dji_sdk_1/dji_sdk/imu` when the stitched
   topic is present instead of the converted-bag `/imu_data_raw`.
3. MSCKF feature model: introduced `MsckfFeatureAccumulator` so monocular
   per-camera temporal features enter the filter, not only stereo-confirmed
   tracks. Landmark tracks are merged across cameras when available.
4. Feature anchor consistency: features are triangulated from their own clone
   observations and refined with three to five Gauss-Newton reprojection steps,
   rather than reusing a body-frame point at an inconsistent anchor pose.
5. Feature-level chi-square gating rejects outlier/stale temporal features.
6. MSCKF parameters moved into `config/system.yaml` under `backend.msckf`;
   CLI overrides are available for quick experiments.
7. Added canonical `trajectory.csv` output in
   `timestamp,x,y,z,qx,qy,qz,qw` order.
8. Restricted cross-camera matching to the two overlapping pairs actually seen
   in the data, `C0-C1` and `C2-C3`.

## Full Dataset Results

| Sequence | ATE_SE3 (m) | ATE_Sim3 (m, ref) | RPE_1m (m) | RPE_5m (m) | RPE_10m (m) | coverage | success |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| eight_noyaw_1 | 241109.33 | 1.5663 | 102.36 | 102.39 | 102.50 | 0.99994 | False |
| eight_noyaw_3_short | 85400.17 | 1.5668 | 50.50 | 50.57 | 50.88 | 0.99982 | False |
| eight_noyaw_4 | 288121.44 | 1.5533 | 109.13 | 109.15 | 109.24 | 0.99979 | True |
| eight_noyaw_5 | 283089.84 | 1.5577 | 110.78 | 110.80 | 110.89 | 0.99986 | False |
| eight_yaw_1 | 67796.28 | 1.5628 | 34.82 | 34.93 | 35.46 | 0.99433 | False |

All sequences completed without NaN/Inf and with high coverage. The remaining
large `ATE_SE3` is scale/drift dominated: `ATE_Sim3` is about 1.55 m, which
means the trajectory shape is closer to groundtruth but the absolute metric
scale and long-term drift are not yet correct.

## Remaining Work

- The filter still starts from an uninitialized IMU bias and gravity direction.
- Feature depth is estimated per update but not kept in a persistent filter
  state, so long tracks cannot tightly constrain scale.
- The camera/IMU time offset and extrinsic perturbation are not estimated.
- Runtime is below real time on this host (RTF about 0.34 to 0.44); a
  deployment-grade version would need feature-count and matching pruning.

## Files

- Per-sequence reports: `result/report_<sequence>.md`
- Full runner logs: `result/run_<sequence>.log`
- Outputs: `output/msckf_<sequence>/trajectory.csv`
