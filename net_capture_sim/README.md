# 4机 Leader/Follower 三维网捕仿真

本仿真实现：
- 4 架无人机（1 Leader + 3 Follower）
- 初始为密集矩形巡航阵列（3D）
- 接收指令后快速扩展为宽间距矩形网阵（增大三维捕获体积）
- 引入四机近距离旋翼下洗气流耦合扰动（Downwash 风场）
- 敌方无人机穿越轨迹
- 输出：mp4 可视化视频 + 一致性收敛图 + csv/json 指标

## 输出文件

运行后在输出目录生成：
- `net_capture_sim_3d.mp4`：三维动态可视化
- `consistency_convergence.png`：一致性与误差收敛图
- `timeseries_metrics.csv`：时序指标
- `summary.json`：汇总指标（含捕获时刻）

## 快速运行

```bash
cd /home/silas/xtdrone-docker/net_capture_sim
python3 sim_leader_follower_net_capture_3d.py --output_dir ./results_3d
```

如果希望网面更“竖”一些（例如 75° 倾角）：

```bash
python3 sim_leader_follower_net_capture_3d.py --output_dir ./results_3d_tilt75 --wide_tilt_deg 75
```

可选参数：
- `--total_time`：仿真总时长（秒）
- `--command_time`：阵型切换指令时间（秒）
- `--seed`：随机种子
- `--wide_tilt_deg`：扩散后网面的目标倾角（相对水平面）
- `--downwash_strength`：下洗风场强度缩放（默认 1.8）
- `--downwash_floor_ratio`：扩散后持续尾流下压比例（默认 0.28）
- `--downwash_floor_xy`：扩散后持续尾流横向扰动幅值（默认 0.12 m/s）
- `--disable_downwash`：关闭下洗风场模型（用于对照实验）

示例：

```bash
python3 sim_leader_follower_net_capture_3d.py --output_dir ./results_case2 --total_time 120 --command_time 30 --seed 7

# 进一步增强下洗扰动场
python3 sim_leader_follower_net_capture_3d.py --output_dir ./results_downwash_strong --downwash_strength 2.1

# 保持扩散后持续下洗扰动（推荐）
python3 sim_leader_follower_net_capture_3d.py --output_dir ./results_downwash_persistent --downwash_floor_ratio 0.32 --downwash_floor_xy 0.14

# 关闭下洗扰动（ablation）
python3 sim_leader_follower_net_capture_3d.py --output_dir ./results_no_downwash --disable_downwash
```

## 指标解释

- `capture_area_xy` + `net_height`：构成三维网捕体积（越大越利于捕获）
- `f*_rel_err_3d`：Follower 相对编队目标误差（3D）
- `f*_est_err_3d`：Follower 对 Leader 广播状态估计误差（3D）
- `estimate_variance`：跟随者之间对 Leader 状态估计的一致性方差（越低越一致）
- `capture_flag`：敌机是否位于三维网捕体积内
- `downwash_intensity`：四机平均下洗扰动强度（m/s）

> 注：该脚本用于控制逻辑与通信一致性验证的快速仿真，不替代 Gazebo 物理细节。
