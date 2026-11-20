# 双目相机标定与物理调平

面向 Jetson/Linux 的双目相机工具，支持针孔与鱼眼镜头：

- 左右相机独立标定、固定内参双目标定及校正预览。
- 实时显示眼距、光心位置差、高度差、深度差和姿态差。
- 提供原始画面、调平辅助线及俯视、侧视、正视、三维视图。

## 安装与编译

需要 C++17、CMake ≥ 3.16、OpenCV ≥ 4.5。

```bash
sudo apt update
sudo apt install -y build-essential cmake pkg-config libopencv-dev

# 可选：中文字体、OpenGL 三维视图及 GStreamer 预览支持
sudo apt install -y libfreetype6-dev fonts-noto-cjk \
  libegl1-mesa-dev libgles2-mesa-dev libx11-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
```

没有 EGL/GLES2 时，空间视图使用 OpenCV 绘制；使用自编译 OpenCV 时，可通过 `-DOpenCV_DIR=/path/to/opencv` 指定配置目录。

## 相机与棋盘

默认左相机 `/dev/video0`、右相机 `/dev/video1`，分辨率 **1920×1080**、帧率 **30 fps**。设备可用 `ls /dev/video*` 查看。

当前棋盘横放为 **12×9 方格、11×8 内角点、单格边长 50 mm**，与程序默认设置一致：

```bash
--board-cols 11 --board-rows 8 --square-size 50
```

竖放时交换列数和行数。格长指单个黑格或白格的边长；更换格长、分辨率或镜头焦距后应重新标定，不能只修改旧参数文件。

## 使用流程

### 1. 左右单目标定

每个相机单独运行一次程序。左相机完成并退出后，再运行右相机。根据镜头类型选择对应命令：

```bash
# 鱼眼镜头
./build/stereo_csi_calibrator --mode fisheye-left --process-scale 0.5
./build/stereo_csi_calibrator --mode fisheye-right --process-scale 0.5

# 针孔镜头
./build/stereo_csi_calibrator --mode mono-left --process-scale 0.5
./build/stereo_csi_calibrator --mode mono-right --process-scale 0.5
```

鱼眼默认保存为 `left_fisheye.yml`、`right_fisheye.yml`；针孔默认保存为 `left_camera.yml`、`right_camera.yml`。采样时移动棋盘，覆盖画面不同位置、距离和倾角，每个姿态停稳；避免反光、模糊及所有样本都正对相机。

单目默认自动采集 15 组，至少 8 组才能计算；达到目标且 RMS ≤ 1 px 时自动保存。可用 `--mono-samples 25` 增加目标组数，或按 `S` 手动采样、`C` 重新计算、`W` 强制保存。下游调平及双目模式会拒绝标定 RMS 超过 2 px 的内参。需要更改单目输出位置时加 `--output`，例如：

```bash
./build/stereo_csi_calibrator --mode mono-left --output left_custom.yml
```

### 2. 物理调平

完成左右单目标定后，固定棋盘并运行：

```bash
# 鱼眼镜头
./build/stereo_csi_calibrator --mode physical-fisheye \
  --left-fisheye left_fisheye.yml --right-fisheye right_fisheye.yml \
  --process-scale 0.5

# 针孔镜头
./build/stereo_csi_calibrator --mode physical \
  --left-mono left_camera.yml --right-mono right_camera.yml \
  --process-scale 0.5
```

两路同时检测到棋盘后显示当前帧的测量估计，无需双目外参或多视角采样。数据无效或过期时眼距显示 `--`。此模式用于观察和调整，`S/C/W` 不执行标定，`R` 不提供校正预览。

### 3. 双目标定与校正

完成对应镜头类型的左右单目标定并固定相机位置后，运行以下一组命令。鱼眼和针孔模式必须使用各自对应的内参文件：

```bash
# 鱼眼双目标定
./build/stereo_csi_calibrator --mode stereo-fisheye \
  --left-fisheye left_fisheye.yml --right-fisheye right_fisheye.yml \
  --process-scale 0.5

# 针孔双目标定
./build/stereo_csi_calibrator --mode stereo \
  --left-mono left_camera.yml --right-mono right_camera.yml \
  --process-scale 0.5
```

让同一棋盘完整出现在两路画面中，自动或按 `S` 采集不同姿态。界面分别显示**样本数**和**有效样本数**：总数统计左右双路采样尝试，包括未识别棋盘、过期帧等无效项；只有时间、尺寸、清晰度、覆盖率、位姿 RMS（默认 ≤ 1 px）和姿态去重检查通过的样本才计为有效并参与计算。拒绝的样本会显示原因。

**有效样本达到 12 组时自动计算外参**，计算期间保持预览。成功后暂停自动采样，按 `R` 检查校正画面、按 `W` 保存为 `stereo_fisheye_extrinsics.yml`；失败后每新增一组有效样本自动重试，也可按 `C` 手动重算（至少 8 组有效样本）。针孔双目标定采用相同规则。检查多个位置和距离的对应点是否上下对齐；水平视差用于测深，应保留。

**调整任一相机的安装位置或角度后，需要重新标定双目外参。**

### 4. 常用运行命令

默认设备为 `/dev/video0` 和 `/dev/video1`，默认采集规格为 1920×1080、30 fps。设备、采集规格、棋盘和输出文件都可以在启动时覆盖，例如：

```bash
./build/stereo_csi_calibrator --mode stereo \
  --left-mono left_camera.yml --right-mono right_camera.yml \
  --left-device /dev/video2 --right-device /dev/video3 \
  --width 1920 --height 1080 --fps 30 \
  --board-cols 11 --board-rows 8 --square-size 50 \
  --process-scale 0.5 --output stereo_extrinsics.yml \
  --capture-dir captures
```

查看程序支持的完整选项和快捷键：

```bash
./build/stereo_csi_calibrator --help
```

模式与参数文件对应关系：

| 操作 | 鱼眼 | 针孔 |
| --- | --- | --- |
| 左单目标定 | `fisheye-left` | `mono-left` |
| 右单目标定 | `fisheye-right` | `mono-right` |
| 物理调平 | `physical-fisheye` | `physical` |
| 双目标定 | `stereo-fisheye` | `stereo` |
| 左右内参参数 | `--left-fisheye` / `--right-fisheye` | `--left-mono` / `--right-mono` |
| 默认内参文件 | `left_fisheye.yml` / `right_fisheye.yml` | `left_camera.yml` / `right_camera.yml` |
| 默认双目结果 | `stereo_fisheye_extrinsics.yml` | `stereo_extrinsics.yml` |

## 常用参数与操作

| 参数 | 默认值 / 用途 |
| --- | --- |
| `--left-device` / `--right-device` | `/dev/video0` / `/dev/video1` |
| `--width` / `--height` / `--fps` | `1920` / `1080` / `30`，须与内参及实际采集规格一致 |
| `--process-scale` | `0.25`；建议先用 `0.5`，提高检测分辨率会增加计算量 |
| `--output` | 自定义标定结果路径，默认按模式命名 |
| `--capture-dir` | `captures`，保存采样图片 |
| `--mono-samples` / `--mono-max-rms` | 单目采集目标组数 `15` / 自动保存 RMS 上限 `1.0 px` |
| `--mono-min-sharpness` | 清晰度阈值，默认 `75` |
| `--mono-min-coverage` / `--mono-max-coverage` | 棋盘画面覆盖率范围，默认 `3%` 至 `55%` |
| `--no-hw-preview` | 使用 OpenCV 窗口预览 |
| `--font` | 自定义中文字体路径 |
| `--help` | 查看完整参数 |

| 按键 | 操作 |
| --- | --- |
| `S` / `C` / `W` | 标定模式：采样 / 计算 / 保存 |
| `R` | 双目标定后切换校正预览；鱼眼单目也支持去畸变预览 |
| `A` / `V` | 调平辅助线开关 / 切换空间视图 |
| `J/L`、`I/K`、`B/N` | 三维视图水平、俯仰、滚转 |
| `U/O` / `0` | 缩放 / 复位视角 |
| `X` / `H` / `E` | 坐标轴 / 参考线 / 误差箭头开关 |
| `Q` / `Esc` | 退出 |

OpenCV 窗口的三维视图支持左键拖动旋转、Shift+左键滚转、中键平移及滚轮缩放。

## 测量基准与精度

- 棋盘坐标为 X 向右、Y 向下、Z 垂直板面。眼距是两光心的三维距离；深度差沿棋盘法线计算，不一定对应安装架的前后差。绝对水平需用水平仪建立棋盘基准。
- 调平合格要求每路位姿 RMS ≤ 2 px、高度及深度差 ≤ 2 mm、总姿态差 ≤ 0.5°，连续 10 次分析通过。鱼眼还要求当前棋盘所有对应角点的原图纵差均 ≤ 2 px。
- 输入图像尺寸必须与标定一致，左右软件接收时间差须 ≤ 100 ms，输入帧年龄须 ≤ 1 s；软件时间检查不能代替硬件曝光同步。
- 低 RMS 或显示合格不代表真实误差必然小于 2 mm。内参偏差、角点噪声和板面不平都会影响单帧结果，应结合外部测量核验。
- 机械齐平不能保证原始鱼眼画面按行对齐；整幅画面对齐应使用双目标定及立体校正。
