# DE100 相机标定

棋盘规格：`9 × 6` 内角点，即 `10 × 7` 个黑白方格。

## 1. 采集照片

默认使用外接摄像头索引 1：

```powershell
.\.venv\Scripts\python.exe tools\camera_calibration\capture_images.py --camera 1 --width 640 --height 480
```

- 看到绿色 `FOUND` 后按空格保存。
- 按 `Q` 或 `Esc` 退出。
- 采集 20～30 张。
- 棋盘应出现在画面中心、四角和四边，并包含不同距离和倾斜角度。
- 每张照片都要完整、清晰地看到棋盘。

若打不开相机，依次尝试 `--camera 0` 和 `--camera 2`。

## 2. 计算标定参数

先用直尺测量手机屏幕上一个方格的实际边长。假设测得 12.4 mm：

```powershell
.\.venv\Scripts\python.exe tools\camera_calibration\calibrate_camera.py --square-mm 12.4
```

结果保存在：

```text
tools/camera_calibration/output/camera_calibration.json
```

标定分辨率必须与后续 AprilTag/PNP 使用的分辨率一致。
