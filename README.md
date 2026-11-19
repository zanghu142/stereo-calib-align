# 双摄像头相对位姿实时显示与物理调平

这是一个面向 Jetson/Linux 的双摄像头标定与调平辅助工具。程序同时显示左右相机画面、实时棋盘检测结果、相机姿态数据，以及左下角的相对空间视图。

`physical`（针孔）和 `physical-fisheye`（鱼眼）物理调平模式使用左右相机各自的单目标定结果建立共同棋盘坐标系，实时计算眼距基线、左右光心中心差、高度差、深度差和姿态差。全部读数来自当前有效帧的左右独立位姿，不需要多视角采样或按键重测。

左下角立体图在支持 EGL/OpenGL ES 2.0 的系统上使用 OpenGL 渲染：正交投影、禁用透视，默认从外部斜俯视角观察完整三维场景，也支持任意方向旋转。棋盘位于画面左侧，双摄、基线、光轴和坐标轴位于右侧，便于观察两台相机光轴是否趋于平行。

## 功能

- 双路 V4L2 相机实时采集。
- 棋盘格角点检测。
- 左右相机独立单目标定与参数文件保存。
- 原始画面与校正画面切换。
- 物理调平辅助线和误差提示。
- 左下角相对空间视图：
  - 俯视图、侧视图、正视图、立体图循环切换。
  - OpenGL 立体图使用轴测正交渲染。
  - 显示棋盘平面、左右相机、双摄基线、光轴、世界坐标轴、水平参考线和误差箭头。
  - 立体图支持鼠标左键拖拽旋转、Shift+左键滚转、中键平移、滚轮缩放。
  - 键盘 `J/L` 水平旋转、`I/K` 俯仰、`B/N` 滚转、`U/O` 缩放、`0` 复位。
  - 俯视严格显示 X-Z，侧视显示 Z-Y，正视显示 X-Y；均使用统一真实毫米坐标。
  - 调平合格需要左右单目标定参数有效、PnP RMS ≤ 2 px、连续 10 帧稳定，并满足高度/深度差 ≤ 2 mm、姿态差 ≤ 0.5°。

## 依赖

Jetson/Linux 建议安装：

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  pkg-config \
  libopencv-dev \
  libfreetype6-dev \
  libegl1-mesa-dev \
  libgles2-mesa-dev \
  libx11-dev \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  fonts-noto-cjk
```

说明：

- `libopencv-dev` 是必须依赖，需要 OpenCV 4.5 或更新版本（本次本地验证使用 4.5.4）。
- `libegl1-mesa-dev` 和 `libgles2-mesa-dev` 用于启用 OpenGL 立体图。
- `libfreetype6-dev` 和 `fonts-noto-cjk` 用于中文文字显示。
- GStreamer 和 X11 是可选能力，系统存在时会自动启用。
- 如果系统没有 EGL/GLES2，程序仍可编译运行，但左下角立体图会退回 OpenCV 2D 绘制。

## 编译

在项目根目录执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

生成程序：

```bash
build/stereo_csi_calibrator
```

## 运行

### 新标定板规格

当前使用 CTGD-12×9-60-45-5 标定板，按实物照片横放：

| 项目 | 当前规格 |
| --- | --- |
| 方格数量 | 横向 12 格、纵向 9 格 |
| 内角点数量 | 横向 11 个、纵向 8 个，共 88 个 |
| 单格边长 | 5 cm，即 50 mm |
| 棋盘图案区域 | 600×450 mm，不含外围留白 |
| 命令参数 | `--board-cols 11 --board-rows 8 --square-size 50` |

`12×9` 是方格数量，程序要求填写内角点数量。竖放、长边沿画面纵向时，使用 `--board-cols 8 --board-rows 11 --square-size 50`。

**程序默认使用新板横放参数：11×8 内角点、格长 50 mm。** 以下命令仍显式指定板规格，便于核对；单目标定界面根据实际启动参数显示内角点数量和格长。使用旧板时需显式指定 `--square-size 20`。

新板与旧板的方格数相同，但单格边长由 20 mm 改为 50 mm。先备份旧参数文件，再重新采集并保存本次结果。程序会检查内参文件中的分辨率、内角点数量和格长是否匹配，不能只把旧文件的 `square_size_mm` 改为 50 来代替重新标定。

### 模式指令速查

| 操作 | 针孔指令 | 鱼眼指令 |
| --- | --- | --- |
| 左相机单目标定 | `--mode mono-left` | `--mode fisheye-left` |
| 右相机单目标定 | `--mode mono-right` | `--mode fisheye-right` |
| 双目外参标定 | `--mode stereo` | `--mode stereo-fisheye` |
| 读取左右内参 | `--left-mono left_camera.yml --right-mono right_camera.yml` | `--left-fisheye left_fisheye.yml --right-fisheye right_fisheye.yml` |
| 物理调平 | `--mode physical` | `--mode physical-fisheye` |

按下面对应小节依次执行左单目、右单目、双目标定命令。所有新板命令均使用 `--board-cols 11 --board-rows 8 --square-size 50`。

### 针孔标定

运行左相机单目标定：

```bash
./build/stereo_csi_calibrator --mode mono-left --width 1920 --height 1080 --board-cols 11 --board-rows 8 --square-size 50 --process-scale 0.5
```

运行右相机单目标定：

```bash
./build/stereo_csi_calibrator \
  --mode mono-right \
  --right-device /dev/video1 \
  --width 1920 --height 1080 \
  --board-cols 11 --board-rows 8 --square-size 50
```

运行针孔双目标定（固定内参，只求左右相机相对位姿）：

```bash
./build/stereo_csi_calibrator --mode stereo \
  --width 1920 --height 1080 \
  --board-cols 11 --board-rows 8 \
  --square-size 50 \
  --left-mono left_camera.yml \
  --right-mono right_camera.yml
```

单目模式会在棋盘静止且姿态与已有样本明显不同时自动取样，默认收集 15 组。从第 8 组开始自动进行初始计算，之后每新增一组更新计算结果；达到目标组数且 RMS 达标后自动保存。计算失败时等待新样本再重试，也可按 `C` 手动重算。按 `C` 计算并在 RMS 达标时保存，或用 `W` 强制保存；左右相机默认分别写入 `left_camera.yml` 和 `right_camera.yml`。保存内容包括分辨率、内参、畸变参数、有效样本数和重投影误差。标定分辨率必须与之后的工作分辨率一致。

### 鱼眼标定

先分别运行左、右鱼眼单目标定：

```bash
./build/stereo_csi_calibrator --mode fisheye-left \
  --left-device /dev/video0 --width 1920 --height 1080 \
  --board-cols 11 --board-rows 8 --square-size 50

./build/stereo_csi_calibrator --mode fisheye-right \
  --right-device /dev/video1 --width 1920 --height 1080 \
  --board-cols 11 --board-rows 8 --square-size 50
```

默认输出分别为 `left_fisheye.yml` 和 `right_fisheye.yml`。单目鱼眼界面同样支持稳定后自动采样、按 `S` 手动采样、`C` 计算、`W` 保存；按 `R` 在原始画面和鱼眼去畸变画面之间切换。鱼眼单目采样按清晰度和棋盘覆盖率筛选，不使用针孔 PnP RMS 做逐帧判断。

左右鱼眼内参都完成后，运行双目鱼眼外参标定：

```bash
./build/stereo_csi_calibrator --mode stereo-fisheye \
  --left-device /dev/video0 --right-device /dev/video1 \
  --width 1920 --height 1080 \
  --board-cols 11 --board-rows 8 --square-size 50 \
  --left-fisheye left_fisheye.yml --right-fisheye right_fisheye.yml
```

该模式会验证两份内参文件的鱼眼模型标记、分辨率和棋盘规格，然后用同步棋盘样本和固定内参计算双目 `R/T` 及鱼眼立体校正映射。默认输出 `stereo_fisheye_extrinsics.yml`，按 `R` 预览校正效果，按 `W` 保存。输出中的平移量 `T` 单位与 `--square-size` 一致；上例为毫米。鱼眼双目文件与针孔 `stereo_extrinsics.yml` 分开保存。

自动采样要求两路都检测到棋盘、画面稳定、位置/姿态相对已有样本有变化，且清晰度和覆盖率合格；两次自动采样间有冷却帧。建议至少采集 12-20 个不同距离、画面位置和倾斜方向的样本，覆盖棋盘中央及边缘，并避免棋盘出画、反光或运动模糊。鱼眼边缘畸变明显，样本应包含靠近画面四周的棋盘姿态。

### 通用参数

指定左右摄像头、分辨率、帧率、棋盘参数：

```bash
./build/stereo_csi_calibrator \
  --left-device /dev/video0 \
  --right-device /dev/video1 \
  --width 1920 \
  --height 1080 \
  --fps 30 \
  --left-mono left_camera.yml \
  --right-mono right_camera.yml \
  --board-cols 11 \
  --board-rows 8 \
  --square-size 50 \
  --process-scale 0.25 \
  --output calibration.yml \
  --capture-dir captures \
  --font /usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc
```

参数说明：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--mode` | `stereo` | `physical`、`physical-fisheye`、`stereo`、`stereo-fisheye`、`mono-left/right`、`fisheye-left/right` |
| `--left-device` | `/dev/video0` | 左相机设备 |
| `--right-device` | `/dev/video1` | 右相机设备 |
| `--width` | `1920` | 采集宽度 |
| `--height` | `1080` | 采集高度 |
| `--fps` | `30` | 采集帧率 |
| `--board-cols` | `11` | 棋盘内角点列数 |
| `--board-rows` | `8` | 棋盘内角点行数 |
| `--square-size` | `50` | 单个棋盘格实际尺寸，单位 mm；新板默认 50，旧板需显式指定 20 |
| `--process-scale` | `0.25` | 棋盘检测缩放比例，数值越小越省算力 |
| `--output` | 按模式决定 | 单目或双目标定结果文件；默认按模式隔离命名 |
| `--left-mono` | `left_camera.yml` | `stereo` / `physical` 读取的左针孔内参 |
| `--right-mono` | `right_camera.yml` | `stereo` / `physical` 读取的右针孔内参 |
| `--left-fisheye` | `left_fisheye.yml` | `stereo-fisheye` / `physical-fisheye` 读取的左鱼眼内参 |
| `--right-fisheye` | `right_fisheye.yml` | `stereo-fisheye` / `physical-fisheye` 读取的右鱼眼内参 |
| `--capture-dir` | `captures` | 样本图片保存目录 |
| `--font` | 自动查找 | 中文字体路径 |
| `--mono-samples` | `15` | 单目标定自动采集的目标样本数，最少 8 |
| `--mono-max-rms` | `1.0` | 自动接受并保存单目标定结果的最大 RMS，单位 px |
| `--mono-min-sharpness` | `75` | 棋盘区域拉普拉斯方差门槛；界面同时显示当前评分和门槛，评分本身不是百分比 |

### 物理调平指令（针孔）

针孔物理四宫格调试界面使用 `physical` 模式，加载本次针孔单目标定文件：

```bash
./build/stereo_csi_calibrator \
  --mode physical \
  --left-device /dev/video0 \
  --right-device /dev/video1 \
  --width 1920 --height 1080 \
  --board-cols 11 --board-rows 8 --square-size 50 \
  --left-mono left_camera.yml \
  --right-mono right_camera.yml
```

### 物理调平指令（鱼眼）

先分别完成左右鱼眼单目标定，再运行：

```bash
./build/stereo_csi_calibrator \
  --mode physical-fisheye \
  --left-device /dev/video0 --right-device /dev/video1 \
  --width 1920 --height 1080 \
  --board-cols 11 --board-rows 8 --square-size 50 \
  --left-fisheye left_fisheye.yml --right-fisheye right_fisheye.yml \
  --process-scale 0.5
```

程序校验两份鱼眼文件的模型、分辨率、棋盘规格和标定 RMS（≤ 2 px），不读取双目外参。两路都看到同一块完整棋盘时，先用鱼眼模型去畸变角点求姿态，再投影回原始鱼眼图像计算位姿 RMS。四宫格显示原始画面、相对空间视图和调平数值，支持原有 `A/V` 与立体图操作；`S/C/W` 不触发此模式的采样、计算或保存，`R` 校正预览不可用。

实时眼距基线（2026-11-18）：面板直接显示当前左右光心的三维距离 `‖C右−C左‖`，单位 mm，光心由 `C=−Rᵀt` 得到。第一组有效位姿即可显示，随新的分析结果更新；调整相机后无需采样或按 `M` 重测。多视角估计、自动采样和结果锁定流程已移除。

实时眼距要求两路位姿有效、内参有效、每路 RMS ≤2 px、原始帧尺寸与时间检查通过，光心坐标有限。丢板、数据过期或质量不足时显示 `--`，不继续展示旧眼距。眼距显示无需等到连续 10 帧调平合格；10 帧条件仍只用于合格判断。

“实时眼距基线（估计）”和实时中心差均为当前画面的测量估计，仍可能受板距、角点噪声、内参误差和板面不平影响。本次按实时显示需求修改，不能保证单帧眼距在所有板距下完全不漂移；实际精度需要实拍及外部测量确认。更新频率取决于棋盘检测和位姿求解速度，不等同于摄像机帧率。

鱼眼物理调平采用联合验收：两路位姿 RMS 均 ≤ 2 px、高度/深度差 ≤ 2 mm、姿态差 ≤ 0.5°，并且当前棋盘所有对应角点的原图纵向差绝对值均 ≤ 2 px，连续 10 帧同时满足后才显示合格。任一角点超限即不通过，不以平均误差掩盖局部大误差。该判定同时控制数值面板、空间视图和辅助线合格颜色；针孔物理模式也统一使用物理容差及连续 10 帧判定，但不要求原图纵差匹配。棋盘丢失、解算失败、原始帧分辨率不符或数据过期时不能显示合格。本模式的位姿 RMS 不用于鱼眼单目/双目标定的采样筛选。

数值面板显示当前棋盘的原图最大/平均绝对纵向差，单位为原始分辨率下的像素。物理达标但原图纵差超限时，明确显示“物理达标，原图纵差未通过”，并提示检查主点差及成像校正，不继续建议把已达标的相机调歪。原图和内参保持原值；该功能是严格验收，不会自动消除画面偏移。

合格只证明本次检测棋盘覆盖区域及距离满足上述阈值，不能保证整幅鱼眼画面、其他距离或运动目标的纵向对应关系。左右主点、焦距和畸变不同，可能导致机械调整无法同时满足物理与原图条件；这种情况会持续显示未通过。需要整幅图按行匹配时应使用鱼眼双目标定和立体校正。相对地面的绝对水平仍需水平仪建立棋盘基准。

左右原始帧的软件接收时间差须 ≤ 100 ms，分析所用原始帧到当前时间须 ≤ 1 s；这不是传感器曝光同步保证，调平时应固定棋盘并停稳相机。缩放检测使用实际图像尺寸与像素中心映射恢复原图坐标，原图精细化及传统检测回退均按角点间距限制窗口，间距不足 8 px 时不使用该次检测。

参数文件会检查模型、RMS、矩阵、格长及 YAML 格式；原图尺寸必须与标定尺寸一致，自动和手动双目采样均检查软件取帧时间。标定失败清除旧有效状态，针孔及鱼眼双目标定均拒绝超过 2 px 的拟合。鱼眼位姿在 PnP 初值后用原始鱼眼像素残差细化，求解及回投使用双精度。

合格阈值不等于实测精度。即使回投 RMS 很小，单帧测量仍可能受角点噪声、内参偏差、板距和板面不平影响，实际误差需结合实拍及外部测量确认。

如果单目标定重投影 RMS 偏大，优先使用 `--process-scale 0.5` 重新采集并标定；`1.0` 会在 1920x1080 全分辨率上逐帧检测棋盘，可能导致预览明显卡顿。确认棋盘参数填写的是内角点数量，并备份旧分辨率或旧格长生成的参数文件，再重新标定。标定文件会保存每张有效样本的 `view_errors`，可用于排查拖累总 RMS 的样本。

查看帮助：

```bash
./build/stereo_csi_calibrator --help
```

## 操作

单目标定模式按键：

| 操作 | 说明 |
| --- | --- |
| `S` | 手动采集当前清晰棋盘画面 |
| `C` | 使用当前样本重新计算，达到 RMS 阈值后保存 |
| `W` | 计算并强制保存当前结果，即使 RMS 高于自动阈值 |
| `Q` 或 `Esc` | 退出单目标定 |

通用按键：

| 操作 | 说明 |
| --- | --- |
| `A` | 开关调平辅助线 |
| `D` | 物理调平模式导出原始帧、内参、角点和投影误差图 |
| `V` | 切换左下角视图：俯视图、侧视图、正视图、立体图 |
| `J/L`、`I/K` | 立体图水平/俯仰旋转 |
| `B/N` | 立体图滚转 |
| `U/O`、`0` | 缩放、恢复默认视角 |
| `S` | 双目标定模式手动采集一组同步样本；单目模式采集当前相机样本 |
| `C` | 计算当前标定；单目模式 RMS 达标时保存，双目模式计算外参 |
| `R` | 在标定结果有效后切换原始画面与校正预览 |
| `W` | 将有效标定结果强制保存到 `--output` |
| `Q` 或 `Esc` | 退出程序 |

OpenGL 立体图显示：

| 操作 | 说明 |
| --- | --- |
| 鼠标左键拖动 | 任意方向旋转 |
| Shift+左键拖动 | 滚转视角 |
| 中键拖动/滚轮 | 平移/缩放 |
| `X` | 显示/隐藏世界坐标轴 |
| `H` | 显示/隐藏水平参考线 |
| `E` | 显示/隐藏误差方向箭头 |

注意：四个视图均使用正交投影。标定板坐标约定为 X 向右、Y 向下、Z 为板面法向；视图能判断双摄相对等高、前后距离和姿态平行。若要判断相对地面的绝对水平，必须先把标定板用水平仪调平。

## 标定流程

1. 连接左右相机，确认设备名，例如：

```bash
ls /dev/video*
```

2. 确认镜头应使用针孔还是鱼眼模型，先分别运行对应的左、右单目标定模式。新板均使用 `--board-cols 11 --board-rows 8 --square-size 50`，先备份旧文件再保存本次结果。
3. 单目采样时保持相机安装和焦距不变，移动棋盘，覆盖不同画面位置、距离及左右/上下倾斜姿态。默认目标为 15 组，建议采集约 15～25 组清晰且姿态不同的画面；需要增加自动采样目标时设置 `--mono-samples 25`。不能所有样本都固定在平视正对姿态。
4. 如需双目外参，针孔使用 `stereo`，鱼眼使用 `stereo-fisheye`，加载本次对应模型的左右内参；两路分辨率和棋盘规格需一致。
5. 标定双目时等待自动采样，或按 `S` 手动采集；棋盘应在两路画面同时完整、清晰可见，同样需要变化棋盘的位置、距离和倾斜姿态。
6. 样本足够后按 `C` 计算标定。双目标定至少需要 8 组有效同步样本。
7. 标定成功后按 `R` 检查校正预览。
8. 按 `W` 保存双目外参文件。单目标定的 `C` 在达到 RMS 阈值时自动保存，`W` 可强制保存。
9. 物理调平时，针孔使用 `physical`，鱼眼使用 `physical-fisheye`，加载本次对应模型的左右单目结果，将标定板固定为相机平视的正面基准，再调整左右相机。该步骤不需要先完成双目标定。

## 物理调平建议

调平目标是让两台相机的光轴尽量平行，同时减少高度差和横滚差。

- 当前场景为摄像头平视标定板。物理调平时固定板子，并用水平仪确认板面的竖直度及横向边缘的水平；相机平视本身不能证明已相对地面调平。
- 观察左右相机画面中的水平辅助线，优先让棋盘在两路画面中的高度接近。
- 观察角点连线和姿态差值，逐步调整相机俯仰、航向和横滚。
- 左下角 OpenGL 立体图用于辅助理解空间关系：棋盘在左侧，左右相机在右侧，光轴指向棋盘方向，基线连接两台相机。

## 常见问题

### 找不到相机

确认设备是否存在：

```bash
ls /dev/video*
```

然后用 `--left-device` 和 `--right-device` 指定正确设备。

### 没有中文

安装中文字体：

```bash
sudo apt install -y fonts-noto-cjk
```

必要时用 `--font` 指定字体文件。

### OpenGL 立体图没有启用

确认 EGL 和 GLES2 开发包已安装：

```bash
sudo apt install -y libegl1-mesa-dev libgles2-mesa-dev
```

重新运行 CMake：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

### 编译找不到 OpenCV

安装 OpenCV 开发包：

```bash
sudo apt install -y libopencv-dev
```

如果使用自编译 OpenCV，需要给 CMake 指定 `OpenCV_DIR`。
