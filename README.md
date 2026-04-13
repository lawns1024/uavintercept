# XTDrone Docker 快速部署（RTX 4060 / Ubuntu 20.04 / WSL2 适配）

本目录提供了三件事：

1. 导入 XTDrone 镜像
2. 启动 GPU + Gazebo 可视化容器
3. 配置宿主机与容器 ROS 通信

> 说明：你当前是 WSL2 + RTX 4060，宿主机实际驱动分支由 Windows 侧控制，不必强行降到 470。
> 只要 `docker run --gpus all ... nvidia-smi` 正常，就可以继续。

---

## 0) 一次性准备

```bash
cd /home/silas/xtdrone-docker
chmod +x *.sh
```

---

## 1) 导入 XTDrone 镜像（只需一次）

从你下载的镜像 tar 导入：

```bash
cd /home/silas/xtdrone-docker
./import_xtdrone_image.sh /path/to/xtdrone-image.tar
```

如果导入后标签不是 `xtdrone:1.3`，可手动改标签：

```bash
sudo docker images
sudo docker tag <IMAGE_ID> xtdrone:1.3
```

---

## 2) 启动容器

```bash
cd /home/silas/xtdrone-docker
./start_xtdrone_container.sh
```

默认：
- 镜像：`xtdrone:1.3`
- 容器名：`xtdrone_sim`
- 工作目录映射：`$HOME/XTDrone_ws -> /workspace`

你也可以临时指定：

```bash
IMAGE=xtdrone:1.3 CONTAINER_NAME=xtdrone_sim HOST_WORKSPACE=$HOME/XTDrone_ws ./start_xtdrone_container.sh
```

---

## 3) 配置 ROS 通信（宿主机 <-> 容器）

容器启动后执行：

```bash
cd /home/silas/xtdrone-docker
./configure_ros_bridge.sh xtdrone_sim
```

脚本会写入：
- 宿主机 `~/.bashrc`
- 容器内 `~/.bashrc`

然后分别在宿主机和容器里执行：

```bash
source ~/.bashrc
```

---

## 4) 快速验证

```bash
sudo docker run --rm --gpus all nvidia/cuda:11.0.3-base-ubuntu20.04 nvidia-smi
```

若能正常看到 GPU 列表，说明 NVIDIA-Docker 链路正常。

---

## 5) 基于 XTDrone 的 Gazebo 3D 仿真（不加下洗气流）

新增了一键脚本：`run_gazebo_3d_no_downwash.sh`

- 使用 XTDrone 默认 `px4/indoor1.launch` 启动 Gazebo 3D
- 不额外加载下洗/风场插件（即“无下洗气流增强项”）

先赋予执行权限：

```bash
cd /home/silas/xtdrone-docker
chmod +x run_gazebo_3d_no_downwash.sh
```

直接启动（GUI）：

```bash
cd /home/silas/xtdrone-docker
./run_gazebo_3d_no_downwash.sh
```

无界面快速验证（30 秒自动退出）：

```bash
cd /home/silas/xtdrone-docker
GUI=false SIM_TIMEOUT=30 INTERACTIVE=0 ./run_gazebo_3d_no_downwash.sh
```

可选环境变量：

- `CONTAINER_NAME`：默认 `xtdrone_sim`
- `LAUNCH_FILE`：默认 `indoor1.launch`
- `XT_LAUNCH_DIR`：默认自动探测（常见为 `/root/XTDrone/sitl_config/launch`）
- `GUI`：`true/false`
- `SIM_TIMEOUT`：>0 时自动超时退出

---

## 6) 四架无人机室外“起飞-前飞-散开”示意（自动飞行，无键盘）

新增脚本：

- `run_xtdrone_outdoor_4uav_spread_demo.sh`（宿主机一键启动）
- `XTDrone/coordination/four_uav_spread_demo.py`（容器内自动飞行程序）

特性：

- 使用你生成的 `coordination/launch_generator/multi_vehicle.launch`
- 室外场景（你当前 launch 为 `outdoor2.world`）
- 不需要再开“通信终端”和“键盘控制终端”
- 自动执行：四机起飞 → 前飞一段距离 → 扩散成围捕示意阵型 → 保持，方便录屏

启动：

```bash
cd /home/silas/xtdrone-docker
chmod +x run_xtdrone_outdoor_4uav_spread_demo.sh
./run_xtdrone_outdoor_4uav_spread_demo.sh
```

可选：

- `STARTUP_WAIT`：仿真启动等待秒数（默认 18）
- `LAUNCH_FILE`：默认 `multi_vehicle.launch`
- `LAUNCH_DIR`：默认自动指向 `.../coordination/launch_generator`
- `GUI`：`true/false`

### 若多机控制链路不稳定：使用“视觉示意版”

当你只需要录屏展示（四机起飞、各飞各的、最终形成编队外观），可使用 Gazebo 模型轨迹驱动版：

- `run_xtdrone_outdoor_4uav_visual_demo.sh`
- `XTDrone/coordination/four_uav_visual_demo_gazebo.py`

该版本不依赖 `OFFBOARD`/通信桥接稳定性，重点保证画面展示稳定。

```bash
cd /home/silas/xtdrone-docker
chmod +x run_xtdrone_outdoor_4uav_visual_demo.sh
./run_xtdrone_outdoor_4uav_visual_demo.sh
```

---

## 常见问题

1. `docker: permission denied /var/run/docker.sock`
   - 重新登录终端（你已加入 docker 组），或临时在命令前加 `sudo`。

2. Gazebo 无法显示
   - 在宿主机执行：

```bash
xhost +local:root
```

3. Gazebo 残留进程冲突

```bash
killall -9 gzclient
killall -9 gzserver
```

---

## 7) 我们做的仿真内容（项目摘要）

本仓库当前包含三类仿真工作，面向“多机协同拦截/围捕”场景：

1. Docker + XTDrone 一键化复现实验环境
2. Gazebo 四机室外协同演示（自动起飞、前飞、扩散围捕示意）
3. 4 机 Leader/Follower 三维网捕快速仿真（含下洗扰动建模与对照实验）

### 7.1 Gazebo 协同演示

- 脚本：`run_xtdrone_outdoor_4uav_spread_demo.sh`
- 控制程序：`XTDrone/coordination/four_uav_spread_demo.py`
- 目标：在可视化环境中展示四机从起飞到扩散成围捕阵型的完整流程

稳定录屏版本（弱依赖通信链路）：

- 脚本：`run_xtdrone_outdoor_4uav_visual_demo.sh`
- 控制程序：`XTDrone/coordination/four_uav_visual_demo_gazebo.py`

### 7.2 三维网捕与通信一致性仿真

- 目录：`net_capture_sim/`
- 主脚本：`net_capture_sim/sim_leader_follower_net_capture_3d.py`
- 核心机制：
   - 四机由密集巡航阵列切换至宽间距网阵
   - 敌机穿越轨迹建模
   - Follower 仅通过 Leader 广播状态做估计与一致性收敛
   - 引入近距旋翼下洗扰动（Downwash），支持强度和持续项配置

### 7.3 指标与产物

每次仿真输出：

- `summary.json`：关键汇总指标
- `timeseries_metrics.csv`：时序指标
- `net_capture_sim_3d.mp4`：过程可视化视频
- `consistency_convergence.png`：一致性与误差收敛曲线

关键指标说明：

- `capture_time_s`：达到捕获条件的时间（越小越好）
- `peak_capture_area_xy`：网阵在 XY 平面的最大覆盖面积（越大越好）
- `mean_rel_error_3d_after_command`：阵型切换后相对误差均值（越小越好）
- `mean_est_error_3d_after_command`：状态估计误差均值（越小越好）
- `estimate_variance_final`：一致性方差（越小越一致）

### 7.4 已完成实验（当前结果目录）

- `results_3d`：基础三维网捕版本
- `results_3d_downwash`：加入下洗扰动
- `results_3d_downwash_boosted`：增强下洗强度
- `results_3d_downwash_persistent`：扩散后持续下洗
- `results_3d_tilt_cn`：倾斜网面设定
- `results_3d_v2`：大覆盖快速捕获版本

基于当前 `summary.json` 的简要对比：

| 实验目录 | capture_time_s | peak_capture_area_xy | mean_rel_error_3d_after_command |
|---|---:|---:|---:|
| `results_3d` | 15.8 | 661.12 | 0.1191 |
| `results_3d_downwash` | 31.8 | 387.65 | 0.4775 |
| `results_3d_downwash_boosted` | 31.8 | 389.23 | 0.5646 |
| `results_3d_downwash_persistent` | 31.7 | 388.30 | 0.4859 |
| `results_3d_tilt_cn` | 31.8 | 387.65 | 0.4773 |
| `results_3d_v2` | 17.2 | 670.84 | 0.4644 |

结论性观察（阶段性）：

1. 下洗扰动会显著拉长捕获时间并提升编队误差。
2. 增大网阵覆盖（如 `results_3d_v2`）可以明显缩短捕获时间。
3. 一致性方差在各组实验末期均收敛到约 `5.5e-4`，通信一致性算法稳定。

---

## 8) 复现实验建议流程

1. 先按第 1~4 节完成 Docker 与 GPU/Gazebo 链路验证。
2. 运行第 6 节四机室外演示脚本，确认多机流程可视化正常。
3. 进入 `net_capture_sim/` 运行 3D 网捕脚本，修改下洗参数做对照实验。
4. 对比各 `summary.json` 与 `timeseries_metrics.csv`，统一记录实验结论。
