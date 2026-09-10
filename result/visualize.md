1. 先启动 Xlaunch/VcXsrv，选择 Multiple windows -> Start no client，在 Extra settings 里勾选 Disable access control。
2. 在 PowerShell 里运行：
``` bash
docker run --rm -it `
  -e DISPLAY=host.docker.internal:0.0 `
  -v "C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\Sphere_VIO:/repo" `
  -v "C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\D2dataset\quadcam_7inch_n3_2023_1_14:/data" `
  sphere_vio:noetic `
  python3 /repo/scripts/visualize_trajectory.py `
    --groundtruth /data/eight_noyaw_3_short-groundtruth.txt `
    --trajectory /repo/output/msckf_noyaw_3_short/trajectory.csv
```
按 Q 或 Esc 退出。要换其他序列，把 eight_noyaw_3_short 改成 eight_noyaw_1、eight_noyaw_4、eight_noyaw_5 或 eight_yaw_1 即可。

e.g.
``` bash
docker run --rm -it `
  -e DISPLAY=host.docker.internal:0.0 `
  -v "C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\Sphere_VIO:/repo" `
  -v "C:\HHQ_STUDY\RM_QY\Drone\oiv_and_slam\D2dataset\quadcam_7inch_n3_2023_1_14:/data" `
  sphere_vio:noetic `
  python3 /repo/scripts/visualize_trajectory.py `
    --groundtruth /data/eight_yaw_1-groundtruth.txt `
    --trajectory /repo/output/msckf_noyaw_3_short/trajectory.csv
```