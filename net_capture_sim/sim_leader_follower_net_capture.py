#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


@dataclass
class SimConfig:
    dt: float = 0.1
    total_time: float = 100.0
    command_time: float = 35.0
    dense_w: float = 8.0
    dense_h: float = 6.0
    wide_w: float = 30.0
    wide_h: float = 22.0
    transition_tau: float = 5.0
    leader_speed_cap: float = 2.2
    follower_speed_cap: float = 3.8
    kp_leader: float = 1.3
    kp_follower: float = 1.8
    obs_gain: float = 3.5
    consensus_gain: float = 1.2
    packet_loss_prob: float = 0.08
    noise_std: float = 0.08
    delay_steps: int = 3
    seed: int = 42


def rect_patrol_reference(t: float, center=(0.0, 0.0), w: float = 14.0, h: float = 10.0, speed: float = 1.2) -> np.ndarray:
    perimeter = 2.0 * (w + h)
    s = (speed * t) % perimeter
    cx, cy = center
    x0, y0 = cx - w / 2, cy - h / 2

    if s < w:
        x, y = x0 + s, y0
    elif s < w + h:
        x, y = x0 + w, y0 + (s - w)
    #!/usr/bin/env python3
    from __future__ import annotations

    import argparse
    import json
    import math
    from dataclasses import dataclass
    from pathlib import Path
    from typing import Dict

    import cv2
    import matplotlib.pyplot as plt
    import numpy as np
    import pandas as pd


    @dataclass
    class SimConfig:
        dt: float = 0.1
        total_time: float = 90.0
        command_time: float = 15.0
        dense_w: float = 8.0
        dense_h: float = 6.0
        wide_w: float = 30.0
        wide_h: float = 22.0
        transition_tau: float = 4.0
        leader_speed_cap: float = 2.4
        follower_speed_cap: float = 4.2
        kp_leader: float = 1.35
        kp_follower: float = 1.95
        obs_gain: float = 3.0
        consensus_gain: float = 1.4
        packet_loss_prob: float = 0.08
        noise_std_xyz: float = 0.08
        delay_steps: int = 3
        net_center_z: float = 8.0
        dense_net_height: float = 2.0
        wide_net_height: float = 6.0
        seed: int = 42


    def rect_patrol_reference_3d(
        t: float,
        z: float,
        center=(0.0, 0.0),
        w: float = 14.0,
        h: float = 10.0,
        speed: float = 1.25,
    ) -> np.ndarray:
        perimeter = 2.0 * (w + h)
        s = (speed * t) % perimeter
        cx, cy = center
        x0, y0 = cx - w / 2, cy - h / 2

        if s < w:
            x, y = x0 + s, y0
        elif s < w + h:
            x, y = x0 + w, y0 + (s - w)
        elif s < 2 * w + h:
            x, y = x0 + w - (s - (w + h)), y0 + h
        else:
            x, y = x0, y0 + h - (s - (2 * w + h))

        z_ref = z + 0.2 * math.sin(0.18 * t)
        return np.array([x, y, z_ref], dtype=np.float64)


    def saturate(v: np.ndarray, vmax: float) -> np.ndarray:
        n = np.linalg.norm(v)
        if n <= vmax or n < 1e-12:
            return v
        return v * (vmax / n)


    def polygon_area(poly_xy: np.ndarray) -> float:
        x = poly_xy[:, 0]
        y = poly_xy[:, 1]
        return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


    def order_quad_xy(points_xy: np.ndarray) -> np.ndarray:
        c = points_xy.mean(axis=0)
        ang = np.arctan2(points_xy[:, 1] - c[1], points_xy[:, 0] - c[0])
        return points_xy[np.argsort(ang)]


    def point_in_convex_quad_xy(p_xy: np.ndarray, quad_xy: np.ndarray) -> bool:
        def cross(a, b, c):
            return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

        signs = []
        for i in range(4):
            a = quad_xy[i]
            b = quad_xy[(i + 1) % 4]
            signs.append(cross(a, b, p_xy))
        return all(s >= 0 for s in signs) or all(s <= 0 for s in signs)


    def interpolate_offsets_3d(t: float, cfg: SimConfig) -> tuple[np.ndarray, float]:
        dense = np.array([
            [-cfg.dense_w / 2, cfg.dense_h / 2, 0.0],
            [cfg.dense_w / 2, cfg.dense_h / 2, 0.0],
            [cfg.dense_w / 2, -cfg.dense_h / 2, 0.0],
            [-cfg.dense_w / 2, -cfg.dense_h / 2, 0.0],
        ], dtype=np.float64)
        wide = np.array([
            [-cfg.wide_w / 2, cfg.wide_h / 2, 0.0],
            [cfg.wide_w / 2, cfg.wide_h / 2, 0.0],
            [cfg.wide_w / 2, -cfg.wide_h / 2, 0.0],
            [-cfg.wide_w / 2, -cfg.wide_h / 2, 0.0],
        ], dtype=np.float64)

        if t <= cfg.command_time:
            alpha = 0.0
        else:
            alpha = 1.0 - math.exp(-(t - cfg.command_time) / cfg.transition_tau)

        offsets = dense * (1 - alpha) + wide * alpha
        net_height = cfg.dense_net_height * (1 - alpha) + cfg.wide_net_height * alpha
        return offsets, net_height


    def enemy_motion_3d(t: float, dt: float, prev: np.ndarray, command_time: float, z_ref: float) -> np.ndarray:
        # Enemy remains separated from leader before command and enters 3D capture volume afterward.
        if t < command_time:
            vx = -1.05 + 0.22 * math.sin(0.13 * t)
            vy = 0.85 * math.sin(0.22 * t + 0.6)
            z = z_ref + 2.8 + 0.7 * math.sin(0.41 * t)
            return np.array([prev[0] + dt * vx, prev[1] + dt * vy, z], dtype=np.float64)

        # Post command: move near formation center with evasive oscillation, gradually descending into the net.
        center_pull_xy = -0.28 * prev[:2]
        evasive_xy = np.array([
            0.45 * math.sin(0.9 * t),
            0.38 * math.cos(1.2 * t),
        ])
        vx, vy = center_pull_xy + evasive_xy

        z_target = z_ref + 0.2 + 0.25 * math.sin(0.5 * t)
        vz = 0.9 * (z_target - prev[2])
        return prev + dt * np.array([vx, vy, vz], dtype=np.float64)


    def simulate(cfg: SimConfig) -> Dict[str, np.ndarray]:
        np.random.seed(cfg.seed)

        n = int(cfg.total_time / cfg.dt) + 1
        t = np.arange(n) * cfg.dt

        # x: [time, drone_id, xyz], 4 drones (0=leader)
        x = np.zeros((n, 4, 3), dtype=np.float64)
        # followers' estimate of leader state (xyz)
        x_hat = np.zeros((n, 3, 3), dtype=np.float64)
        enemy = np.zeros((n, 3), dtype=np.float64)

        init_off, net_h0 = interpolate_offsets_3d(0.0, cfg)
        center0 = np.array([0.0, 0.0, cfg.net_center_z], dtype=np.float64)
        for i in range(4):
            x[0, i] = center0 + init_off[i] + np.random.randn(3) * np.array([0.15, 0.15, 0.05])

        for i in range(3):
            x_hat[0, i] = x[0, 0] + np.random.randn(3) * np.array([0.22, 0.22, 0.08])

        # Enemy starts away from leader, avoiding overlap at origin.
        enemy[0] = np.array([20.0, -10.0, cfg.net_center_z + 2.8], dtype=np.float64)

        neighbors = {0: [1, 2], 1: [0, 2], 2: [0, 1]}

        delay = max(cfg.delay_steps, 0)
        leader_buffer = [x[0, 0].copy() for _ in range(delay + 1)]

        capture_area_xy = np.zeros(n)
        net_height_series = np.zeros(n)
        capture_flag = np.zeros(n, dtype=np.int32)
        rel_err = np.zeros((n, 3))
        est_err = np.zeros((n, 3))
        estimate_var = np.zeros(n)
        enemy_dist_to_center = np.zeros(n)

        for k in range(1, n):
            tk = t[k]

            if tk < cfg.command_time:
                leader_ref = rect_patrol_reference_3d(tk, cfg.net_center_z)
            else:
                leader_ref = np.array([0.0, 0.0, cfg.net_center_z], dtype=np.float64)

            v_leader = cfg.kp_leader * (leader_ref - x[k - 1, 0])
            v_leader = saturate(v_leader, cfg.leader_speed_cap)
            x[k, 0] = x[k - 1, 0] + cfg.dt * v_leader

            # broadcast with delay/loss/noise
            leader_buffer.append(x[k, 0].copy())
            delayed_state = leader_buffer[-(delay + 1)]

            recv = np.random.rand(3) > cfg.packet_loss_prob
            for i in range(3):
                consensus_term = np.zeros(3)
                for j in neighbors[i]:
                    consensus_term += (x_hat[k - 1, j] - x_hat[k - 1, i])

                obs_term = np.zeros(3)
                if recv[i]:
                    meas = delayed_state + np.random.randn(3) * cfg.noise_std_xyz
                    obs_term = meas - x_hat[k - 1, i]

                x_hat[k, i] = x_hat[k - 1, i] + cfg.dt * (
                    cfg.obs_gain * obs_term + cfg.consensus_gain * consensus_term
                )

            offs, net_h = interpolate_offsets_3d(tk, cfg)
            net_height_series[k] = net_h

            for i in range(1, 4):
                target = x_hat[k, i - 1] + (offs[i] - offs[0])
                v = cfg.kp_follower * (target - x[k - 1, i])
                v = saturate(v, cfg.follower_speed_cap)
                x[k, i] = x[k - 1, i] + cfg.dt * v

            enemy[k] = enemy_motion_3d(tk, cfg.dt, enemy[k - 1], cfg.command_time, cfg.net_center_z)

            quad_xy = order_quad_xy(x[k, :, :2])
            area_xy = polygon_area(quad_xy)
            capture_area_xy[k] = area_xy

            inside_xy = point_in_convex_quad_xy(enemy[k, :2], quad_xy)
            z_center = np.mean(x[k, :, 2])
            inside_z = abs(enemy[k, 2] - z_center) <= 0.5 * net_h
            capture_flag[k] = 1 if (inside_xy and inside_z) else 0

            for i in range(1, 4):
                desired_rel = offs[i] - offs[0]
                actual_rel = x[k, i] - x[k, 0]
                rel_err[k, i - 1] = np.linalg.norm(actual_rel - desired_rel)
                est_err[k, i - 1] = np.linalg.norm(x_hat[k, i - 1] - x[k, 0])

            estimate_var[k] = np.mean(np.var(x_hat[k], axis=0))
            enemy_dist_to_center[k] = np.linalg.norm(enemy[k] - np.mean(x[k], axis=0))

        # Capture declared if inside capture volume continuously for >= 1.5s
        win = max(1, int(1.5 / cfg.dt))
        conv = np.convolve(capture_flag, np.ones(win, dtype=np.int32), mode='valid')
        idx = np.where(conv >= win)[0]
        capture_time = None if idx.size == 0 else float(t[idx[0]])

        return {
            't': t,
            'x': x,
            'x_hat': x_hat,
            'enemy': enemy,
            'capture_area_xy': capture_area_xy,
            'net_height': net_height_series,
            'capture_flag': capture_flag,
            'rel_err': rel_err,
            'est_err': est_err,
            'estimate_var': estimate_var,
            'enemy_dist_to_center': enemy_dist_to_center,
            'capture_time': np.array([-1.0 if capture_time is None else capture_time]),
        }


    def save_metrics(data: Dict[str, np.ndarray], out_dir: Path, cfg: SimConfig) -> None:
        t = data['t']
        rel = data['rel_err']
        est = data['est_err']
        x = data['x']
        enemy = data['enemy']

        df = pd.DataFrame({
            'time_s': t,
            'capture_area_xy': data['capture_area_xy'],
            'net_height': data['net_height'],
            'capture_flag': data['capture_flag'],
            'f1_rel_err_3d': rel[:, 0],
            'f2_rel_err_3d': rel[:, 1],
            'f3_rel_err_3d': rel[:, 2],
            'f1_est_err_3d': est[:, 0],
            'f2_est_err_3d': est[:, 1],
            'f3_est_err_3d': est[:, 2],
            'estimate_variance': data['estimate_var'],
            'enemy_x': enemy[:, 0],
            'enemy_y': enemy[:, 1],
            'enemy_z': enemy[:, 2],
            'leader_x': x[:, 0, 0],
            'leader_y': x[:, 0, 1],
            'leader_z': x[:, 0, 2],
            'enemy_center_dist_3d': data['enemy_dist_to_center'],
        })
        df.to_csv(out_dir / 'timeseries_metrics.csv', index=False)

        cmd_idx = np.searchsorted(t, cfg.command_time)
        post = slice(cmd_idx, None)
        summary = {
            'command_time_s': cfg.command_time,
            'capture_time_s': None if data['capture_time'][0] < 0 else float(data['capture_time'][0]),
            'peak_capture_area_xy': float(np.max(data['capture_area_xy'])),
            'peak_net_capture_volume': float(np.max(data['capture_area_xy'] * data['net_height'])),
            'mean_rel_error_3d_after_command': float(np.mean(rel[post])),
            'mean_est_error_3d_after_command': float(np.mean(est[post])),
            'estimate_variance_final': float(data['estimate_var'][-1]),
        }
        (out_dir / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))


    def plot_convergence(data: Dict[str, np.ndarray], out_dir: Path, cfg: SimConfig) -> None:
        t = data['t']
        net_volume = data['capture_area_xy'] * np.maximum(data['net_height'], 1e-6)

        fig, axes = plt.subplots(2, 2, figsize=(12, 8))

        axes[0, 0].plot(t, net_volume, lw=2)
        axes[0, 0].axvline(cfg.command_time, color='r', ls='--', label='command')
        axes[0, 0].set_title('3D net capture volume evolution')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Volume')
        axes[0, 0].legend()
        axes[0, 0].grid(alpha=0.3)

        for i in range(3):
            axes[0, 1].plot(t, data['rel_err'][:, i], label=f'Follower {i+1}')
        axes[0, 1].axvline(cfg.command_time, color='r', ls='--')
        axes[0, 1].set_title('Follower 3D formation tracking error')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('||e_rel|| (m)')
        axes[0, 1].legend()
        axes[0, 1].grid(alpha=0.3)

        for i in range(3):
            axes[1, 0].plot(t, data['est_err'][:, i], label=f'Follower {i+1}')
        axes[1, 0].axvline(cfg.command_time, color='r', ls='--')
        axes[1, 0].set_title('Leader broadcast state estimation error (3D)')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('||xhat - xL|| (m)')
        axes[1, 0].legend()
        axes[1, 0].grid(alpha=0.3)

        axes[1, 1].plot(t, data['estimate_var'], color='tab:purple', lw=2, label='estimate variance')
        axes[1, 1].plot(t, data['enemy_dist_to_center'], color='tab:gray', lw=1.5, label='enemy-center dist')
        axes[1, 1].axvline(cfg.command_time, color='r', ls='--')
        axes[1, 1].set_title('Consensus + enemy relative distance')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].legend()
        axes[1, 1].grid(alpha=0.3)

        fig.tight_layout()
        fig.savefig(out_dir / 'consistency_convergence.png', dpi=180)
        plt.close(fig)


    def render_video(data: Dict[str, np.ndarray], out_dir: Path, cfg: SimConfig, fps: int = 15, frame_skip: int = 2) -> None:
        t = data['t']
        x = data['x']
        enemy = data['enemy']

        fig = plt.figure(figsize=(10, 7), dpi=120)
        ax = fig.add_subplot(111, projection='3d')

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_path = str(out_dir / 'net_capture_sim_3d.mp4')
        writer = cv2.VideoWriter(video_path, fourcc, fps, (1280, 840))
        if not writer.isOpened():
            raise RuntimeError(f'Cannot open video writer: {video_path}')

        capture_start = None
        if data['capture_time'][0] >= 0:
            capture_start = data['capture_time'][0]

        colors = ['tab:red', 'tab:blue', 'tab:green', 'tab:orange']
        labels = ['Leader', 'Follower-1', 'Follower-2', 'Follower-3']

        for k in range(0, len(t), frame_skip):
            ax.clear()
            ax.set_xlim(-32, 32)
            ax.set_ylim(-25, 25)
            ax.set_zlim(3, 15)
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.view_init(elev=26, azim=35)
            ax.set_title('4-UAV Leader/Follower 3D Net-Capture Simulation')

            state_txt = 'Dense patrol formation' if t[k] < cfg.command_time else 'Expanded 3D net formation'
            ax.text2D(0.02, 0.95, f't={t[k]:.1f}s | {state_txt}', transform=ax.transAxes)

            # Drone trajectories and points
            for i in range(4):
                ax.plot(x[:k + 1, i, 0], x[:k + 1, i, 1], x[:k + 1, i, 2], lw=1.2, color=colors[i])
                ax.scatter(x[k, i, 0], x[k, i, 1], x[k, i, 2], s=55, color=colors[i], label=labels[i])

            # Net polygon (top and bottom planes)
            quad_xy = order_quad_xy(x[k, :, :2])
            zc = np.mean(x[k, :, 2])
            net_h = data['net_height'][k]
            z_top = zc + 0.5 * net_h
            z_bot = zc - 0.5 * net_h

            top_poly = np.column_stack([quad_xy, np.full(4, z_top)])
            bot_poly = np.column_stack([quad_xy, np.full(4, z_bot)])
            for poly in [top_poly, bot_poly]:
                pp = np.vstack([poly, poly[0]])
                ax.plot(pp[:, 0], pp[:, 1], pp[:, 2], color='k', lw=1.8)
            for i in range(4):
                ax.plot([top_poly[i, 0], bot_poly[i, 0]], [top_poly[i, 1], bot_poly[i, 1]], [top_poly[i, 2], bot_poly[i, 2]], color='k', lw=1.0, alpha=0.7)

            # Enemy trajectory
            ax.plot(enemy[:k + 1, 0], enemy[:k + 1, 1], enemy[:k + 1, 2], 'm--', lw=1.3, alpha=0.9)
            ax.scatter(enemy[k, 0], enemy[k, 1], enemy[k, 2], s=80, color='magenta', marker='x', label='Enemy UAV')

            inside_xy = point_in_convex_quad_xy(enemy[k, :2], quad_xy)
            inside_z = abs(enemy[k, 2] - zc) <= 0.5 * net_h
            if inside_xy and inside_z:
                ax.text2D(0.02, 0.90, 'Enemy inside 3D capture volume', color='green', transform=ax.transAxes)
            if capture_start is not None and t[k] >= capture_start:
                ax.text2D(0.02, 0.86, f'Capture validated @ {capture_start:.1f}s', color='green', transform=ax.transAxes)

            ax.legend(loc='upper right', fontsize=8)

            fig.canvas.draw()
            w, h = fig.canvas.get_width_height()
            frame = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8).reshape(h, w, 4)
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            frame_bgr = cv2.resize(frame_bgr, (1280, 840), interpolation=cv2.INTER_AREA)
            writer.write(frame_bgr)

        writer.release()
        plt.close(fig)


    def main() -> None:
        parser = argparse.ArgumentParser(description='4-UAV leader/follower 3D net-capture simulation')
        parser.add_argument('--output_dir', type=Path, default=Path('output_net_capture_3d'))
        parser.add_argument('--total_time', type=float, default=90.0)
        parser.add_argument('--command_time', type=float, default=15.0)
        parser.add_argument('--seed', type=int, default=42)
        args = parser.parse_args()

        out_dir = args.output_dir.resolve()
        out_dir.mkdir(parents=True, exist_ok=True)

        cfg = SimConfig(total_time=args.total_time, command_time=args.command_time, seed=args.seed)
        data = simulate(cfg)

        save_metrics(data, out_dir, cfg)
        plot_convergence(data, out_dir, cfg)
        render_video(data, out_dir, cfg)

        summary = json.loads((out_dir / 'summary.json').read_text())
        print('[DONE] 3D simulation outputs generated:')
        print(f'  - video: {out_dir / "net_capture_sim_3d.mp4"}')
        print(f'  - plot: {out_dir / "consistency_convergence.png"}')
        print(f'  - csv : {out_dir / "timeseries_metrics.csv"}')
        print(f'  - json: {out_dir / "summary.json"}')
        print(f'  - capture_time_s: {summary["capture_time_s"]}')


    if __name__ == '__main__':
        main()
