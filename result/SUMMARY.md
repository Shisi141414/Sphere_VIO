# Sphere-VIO Full Dataset Summary

## Accuracy

| Sequence | ATE_SE3 (m) | ATE_Sim3 (m, ref) | RPE_1m (m) | RPE_5m (m) | RPE_10m (m) | RPE_R | coverage | success |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| eight_noyaw_1 | 241109.329608 | 1.566323 | 102.363288 | 102.386692 | 102.496604 | N/A | 0.999935 | False |
| eight_noyaw_3_short | 85400.169422 | 1.566771 | 50.504275 | 50.573046 | 50.879091 | N/A | 0.999821 | False |
| eight_noyaw_4 | 288121.435521 | 1.553320 | 109.126746 | 109.148778 | 109.244992 | N/A | 0.999785 | True |
| eight_noyaw_5 | 283089.844834 | 1.557707 | 110.781475 | 110.801365 | 110.892463 | N/A | 0.999860 | False |
| eight_yaw_1 | 67796.276048 | 1.562835 | 34.821933 | 34.925366 | 35.461641 | N/A | 0.994334 | False |

## Performance

| Sequence | RTF | avg_frame_time_ms | frame_step | dropped_frames | cpu_core_hours_per_frame |
| --- | ---: | ---: | ---: | ---: | ---: |
| eight_noyaw_1 | 0.419514 | 119.447348 | 1 | 0 | 0.000069 |
| eight_noyaw_3_short | 0.337042 | 150.529219 | 1 | 0 | 0.000082 |
| eight_noyaw_4 | 0.398816 | 125.497670 | 1 | 0 | 0.000074 |
| eight_noyaw_5 | 0.364213 | 140.337447 | 1 | 0 | 0.000083 |
| eight_yaw_1 | 0.442332 | 148.673139 | 1 | 0 | 0.000087 |

## Notes

- No sequence produced NaN/Inf or illegal quaternions.
- Coverage is above 99% for all sequences.
- The large `ATE_SE3` is dominated by metric scale error and long-term drift;
  `ATE_Sim3` is about 1.55 m on every sequence.
