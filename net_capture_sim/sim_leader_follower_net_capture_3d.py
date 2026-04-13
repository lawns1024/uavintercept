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
from matplotlib import font_manager
import numpy as np
import pandas as pd

def configure_chinese_font() -> None:
    """Configure matplotlib with an available Chinese font."""
    candidate_files = [
        '/mnt/c/Windows/Fonts/msyh.ttc',
        '/mnt/c/Windows/Fonts/msyhbd.ttc',
        '/mnt/c/Windows/Fonts/simhei.ttf',
        '/mnt/c/Windows/Fonts/simsun.ttc',
        '/mnt/c/Windows/Fonts/NotoSansSC-VF.ttf',
        '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
        '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
    ]

    preferred_names = [
        'Noto Sans CJK SC',
        'Noto Sans SC',
        'Microsoft YaHei',
        'SimHei',
        'WenQuanYi Zen Hei',
    ]

    selected_name = None
    for font_file in candidate_files:
        if Path(font_file).exists():
            try:
                font_manager.fontManager.addfont(font_file)
                selected_name = font_manager.FontProperties(fname=font_file).get_name()
                break
            except Exception:
                continue

    plt.rcParams['font.family'] = 'sans-serif'
    if selected_name:
        plt.rcParams['font.sans-serif'] = [selected_name, 'DejaVu Sans']
    else:
        plt.rcParams['font.sans-serif'] = preferred_names + ['DejaVu Sans']


configure_chinese_font()
plt.rcParams['axes.unicode_minus'] = False


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
    dense_net_sag: float = 0.25
    wide_net_sag: float = 1.10
    dense_tilt_deg: float = 0.0
    wide_tilt_deg: float = 55.0
    capture_surface_threshold: float = 1.00
    pursuit_gain: float = 1.0
    enable_downwash: bool = True
    downwash_strength: float = 1.8
    downwash_radial_strength: float = 0.35
    downwash_sigma_r: float = 5.2
    downwash_sigma_z: float = 2.4
    downwash_max_airspeed: float = 2.0
    downwash_floor_ratio: float = 0.28
    downwash_floor_xy: float = 0.12
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

    # Tilt net plane relative to horizontal plane (rotate around x-axis).
    tilt_deg = cfg.dense_tilt_deg * (1 - alpha) + cfg.wide_tilt_deg * alpha
    tilt_rad = math.radians(tilt_deg)
    c, s = math.cos(tilt_rad), math.sin(tilt_rad)
    rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, c, -s],
        [0.0, s, c],
    ], dtype=np.float64)
    offsets = offsets @ rx.T

    net_sag = cfg.dense_net_sag * (1 - alpha) + cfg.wide_net_sag * alpha
    return offsets, net_sag


def expansion_alpha(t: float, cfg: SimConfig) -> float:
    if t <= cfg.command_time:
        return 0.0
    return 1.0 - math.exp(-(t - cfg.command_time) / cfg.transition_tau)


def build_net_surface(corners_xyz: np.ndarray, sag: float, res: int = 17) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Build a deformable 2D bilinear net surface from 4 corner UAV positions.

    Returns:
      X, Y, Z mesh and ordered corner points used for construction.
    """
    ordered_xy = order_quad_xy(corners_xyz[:, :2])

    # map ordered xy back to full xyz corners
    ordered = []
    for p in ordered_xy:
        idx = np.argmin(np.linalg.norm(corners_xyz[:, :2] - p[None, :], axis=1))
        ordered.append(corners_xyz[idx])
    ordered = np.asarray(ordered, dtype=np.float64)

    p00, p10, p11, p01 = ordered
    us = np.linspace(0.0, 1.0, res)
    vs = np.linspace(0.0, 1.0, res)
    X = np.zeros((res, res), dtype=np.float64)
    Y = np.zeros((res, res), dtype=np.float64)
    Z = np.zeros((res, res), dtype=np.float64)

    for i, u in enumerate(us):
        for j, v in enumerate(vs):
            p = (
                (1 - u) * (1 - v) * p00
                + u * (1 - v) * p10
                + u * v * p11
                + (1 - u) * v * p01
            )
            # sag term makes the net a mutable curved 2D surface
            sag_term = 4.0 * u * (1 - u) * v * (1 - v)
            p[2] -= sag * sag_term
            X[i, j], Y[i, j], Z[i, j] = p

    return X, Y, Z, ordered


def enemy_motion_3d(
    t: float,
    dt: float,
    prev: np.ndarray,
    command_time: float,
    z_ref: float,
    formation_center: np.ndarray,
    leader_pos: np.ndarray,
) -> np.ndarray:
    if t < command_time:
        vx = -1.05 + 0.22 * math.sin(0.13 * t)
        vy = 0.85 * math.sin(0.22 * t + 0.6)
        z = z_ref + 2.8 + 0.7 * math.sin(0.41 * t)
        return np.array([prev[0] + dt * vx, prev[1] + dt * vy, z], dtype=np.float64)

    escape = prev[:2] - formation_center[:2]
    n = np.linalg.norm(escape)
    if n < 1e-9:
        escape_dir = np.array([1.0, 0.0], dtype=np.float64)
    else:
        escape_dir = escape / n

    # enemy actively escapes; formation must actively pursue to capture
    center_pull_xy = 0.55 * escape_dir - 0.28 * (prev[:2] - formation_center[:2])
    evasive_xy = np.array([
        0.45 * math.sin(0.9 * t),
        0.38 * math.cos(1.2 * t),
    ])
    vx, vy = center_pull_xy + evasive_xy

    # keep standoff from leader to avoid unrealistic overlap
    rel_l = prev[:2] - leader_pos[:2]
    d_l = np.linalg.norm(rel_l)
    if d_l < 4.5:
        if d_l < 1e-9:
            rel_dir = np.array([1.0, 0.0], dtype=np.float64)
        else:
            rel_dir = rel_l / d_l
        repel = 1.8 * (4.5 - d_l) * rel_dir
        vx += repel[0]
        vy += repel[1]

    z_target = z_ref + 1.0 + 0.4 * math.sin(0.6 * t)
    vz = 0.9 * (z_target - prev[2])
    return prev + dt * np.array([vx, vy, vz], dtype=np.float64)


def compute_downwash_disturbance(
    positions_xyz: np.ndarray,
    cfg: SimConfig,
    t: float,
    alpha: float,
) -> np.ndarray:
    """Approximate near-field rotor downwash coupling among UAVs.

    Model:
    - Downwash coupling exists for close neighbors at different/same heights.
    - Influence decays with horizontal offset (Gaussian) and vertical separation (exponential).
    - Upper-to-lower interaction is stronger; same/lower layers keep weak recirculation coupling.
    - Disturbance includes downward component + small radial push.
    """
    n_uav = positions_xyz.shape[0]
    disturb = np.zeros((n_uav, 3), dtype=np.float64)
    if not cfg.enable_downwash:
        return disturb

    sigma_r = max(cfg.downwash_sigma_r, 1e-6)
    sigma_z = max(cfg.downwash_sigma_z, 1e-6)

    for i in range(n_uav):
        for j in range(n_uav):
            if i == j:
                continue

            rel = positions_xyz[i] - positions_xyz[j]
            r_xy = np.linalg.norm(rel[:2])
            dz = positions_xyz[j, 2] - positions_xyz[i, 2]  # dz>0 => i is below j

            # core attenuation terms
            weight_r = math.exp(-(r_xy ** 2) / (2.0 * sigma_r ** 2))
            weight_z = math.exp(-abs(dz) / sigma_z)

            # vertical bias: stronger when source is above target, weaker otherwise
            upper_bias = 1.0 / (1.0 + math.exp(-1.6 * dz / sigma_z))
            vertical_factor = 0.25 + 0.75 * upper_bias

            weight = weight_r * weight_z * vertical_factor

            # dominant downward component
            disturb[i, 2] += -cfg.downwash_strength * weight

            # weaker radial spreading in xy plane
            if r_xy > 1e-6:
                disturb[i, :2] += cfg.downwash_radial_strength * weight * (rel[:2] / r_xy)

    for i in range(n_uav):
        disturb[i] = saturate(disturb[i], cfg.downwash_max_airspeed)

    # persistent wake field after expansion: even with large spacing,
    # urban low-altitude turbulence + wake memory still introduces weak disturbance.
    if alpha > 0.0:
        floor_down = cfg.downwash_strength * cfg.downwash_floor_ratio * alpha
        floor_xy = cfg.downwash_floor_xy * alpha
        for i in range(n_uav):
            phase = t + 0.9 * i
            disturb[i, 0] += floor_xy * math.sin(0.8 * phase)
            disturb[i, 1] += floor_xy * math.cos(0.7 * phase)
            disturb[i, 2] += -floor_down
            disturb[i] = saturate(disturb[i], cfg.downwash_max_airspeed)

    return disturb


def simulate(cfg: SimConfig) -> Dict[str, np.ndarray]:
    np.random.seed(cfg.seed)

    n = int(cfg.total_time / cfg.dt) + 1
    t = np.arange(n) * cfg.dt

    x = np.zeros((n, 4, 3), dtype=np.float64)
    x_hat = np.zeros((n, 3, 3), dtype=np.float64)
    enemy = np.zeros((n, 3), dtype=np.float64)

    init_off, _ = interpolate_offsets_3d(0.0, cfg)
    center0 = np.array([0.0, 0.0, cfg.net_center_z], dtype=np.float64)
    for i in range(4):
        x[0, i] = center0 + init_off[i] + np.random.randn(3) * np.array([0.15, 0.15, 0.05])

    for i in range(3):
        x_hat[0, i] = x[0, 0] + np.random.randn(3) * np.array([0.22, 0.22, 0.08])

    enemy[0] = np.array([20.0, -10.0, cfg.net_center_z + 2.8], dtype=np.float64)

    neighbors = {0: [1, 2], 1: [0, 2], 2: [0, 1]}

    delay = max(cfg.delay_steps, 0)
    leader_buffer = [x[0, 0].copy() for _ in range(delay + 1)]

    capture_area_xy = np.zeros(n)
    net_sag_series = np.zeros(n)
    surface_distance = np.zeros(n)
    capture_flag = np.zeros(n, dtype=np.int32)
    rel_err = np.zeros((n, 3))
    est_err = np.zeros((n, 3))
    estimate_var = np.zeros(n)
    enemy_dist_to_center = np.zeros(n)
    leader_enemy_dist = np.zeros(n)
    downwash_intensity = np.zeros(n)
    leader_enemy_dist[0] = np.linalg.norm(enemy[0] - x[0, 0])

    for k in range(1, n):
        tk = t[k]

        offs, net_sag = interpolate_offsets_3d(tk, cfg)
        net_sag_series[k] = net_sag
        alpha = expansion_alpha(tk, cfg)
        downwash = compute_downwash_disturbance(x[k - 1], cfg, tk, alpha)
        downwash_intensity[k] = float(np.mean(np.linalg.norm(downwash, axis=1)))

        if tk < cfg.command_time:
            leader_ref = rect_patrol_reference_3d(tk, cfg.net_center_z)
        else:
            # active pursuit: drive net center toward enemy, not waiting in place
            net_center = x[k - 1, 0] - offs[0]
            desired_center = net_center + cfg.pursuit_gain * (enemy[k - 1] - net_center)
            desired_center[2] = cfg.net_center_z + 0.4 * math.sin(0.08 * tk)
            leader_ref = desired_center + offs[0]

        v_leader = cfg.kp_leader * (leader_ref - x[k - 1, 0])
        v_leader = saturate(v_leader, cfg.leader_speed_cap)
        x[k, 0] = x[k - 1, 0] + cfg.dt * (v_leader + downwash[0])

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

        for i in range(1, 4):
            target = x_hat[k, i - 1] + (offs[i] - offs[0])
            v = cfg.kp_follower * (target - x[k - 1, i])
            v = saturate(v, cfg.follower_speed_cap)
            x[k, i] = x[k - 1, i] + cfg.dt * (v + downwash[i])

        formation_center = np.mean(x[k], axis=0)
        enemy[k] = enemy_motion_3d(
            tk,
            cfg.dt,
            enemy[k - 1],
            cfg.command_time,
            cfg.net_center_z,
            formation_center,
            x[k, 0],
        )

        Xs, Ys, Zs, ordered_corners = build_net_surface(x[k], sag=net_sag, res=17)
        quad_xy = ordered_corners[:, :2]
        capture_area_xy[k] = polygon_area(quad_xy)

        inside_xy = point_in_convex_quad_xy(enemy[k, :2], quad_xy)
        surface_pts = np.column_stack([Xs.ravel(), Ys.ravel(), Zs.ravel()])
        surface_distance[k] = float(np.min(np.linalg.norm(surface_pts - enemy[k][None, :], axis=1)))
        capture_flag[k] = 1 if (inside_xy and surface_distance[k] <= cfg.capture_surface_threshold) else 0

        for i in range(1, 4):
            desired_rel = offs[i] - offs[0]
            actual_rel = x[k, i] - x[k, 0]
            rel_err[k, i - 1] = np.linalg.norm(actual_rel - desired_rel)
            est_err[k, i - 1] = np.linalg.norm(x_hat[k, i - 1] - x[k, 0])

        estimate_var[k] = np.mean(np.var(x_hat[k], axis=0))
        enemy_dist_to_center[k] = np.linalg.norm(enemy[k] - np.mean(x[k], axis=0))
        leader_enemy_dist[k] = np.linalg.norm(enemy[k] - x[k, 0])

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
        'net_sag': net_sag_series,
        'surface_distance': surface_distance,
        'capture_flag': capture_flag,
        'rel_err': rel_err,
        'est_err': est_err,
        'estimate_var': estimate_var,
        'enemy_dist_to_center': enemy_dist_to_center,
        'leader_enemy_dist': leader_enemy_dist,
        'downwash_intensity': downwash_intensity,
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
        'net_sag': data['net_sag'],
        'surface_distance': data['surface_distance'],
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
        'leader_enemy_dist_3d': data['leader_enemy_dist'],
        'downwash_intensity': data['downwash_intensity'],
    })
    df.to_csv(out_dir / 'timeseries_metrics.csv', index=False)

    cmd_idx = np.searchsorted(t, cfg.command_time)
    post = slice(cmd_idx, None)
    summary = {
        'command_time_s': cfg.command_time,
        'capture_time_s': None if data['capture_time'][0] < 0 else float(data['capture_time'][0]),
        'peak_capture_area_xy': float(np.max(data['capture_area_xy'])),
        'min_surface_distance': float(np.min(data['surface_distance'])),
        'mean_surface_distance_after_command': float(np.mean(data['surface_distance'][post])),
        'mean_rel_error_3d_after_command': float(np.mean(rel[post])),
        'mean_est_error_3d_after_command': float(np.mean(est[post])),
        'mean_leader_enemy_dist_after_command': float(np.mean(data['leader_enemy_dist'][post])),
        'min_leader_enemy_dist': float(np.min(data['leader_enemy_dist'])),
        'estimate_variance_final': float(data['estimate_var'][-1]),
        'mean_downwash_intensity': float(np.mean(data['downwash_intensity'][post])),
        'peak_downwash_intensity': float(np.max(data['downwash_intensity'])),
    }
    (out_dir / 'summary.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False))


def plot_convergence(data: Dict[str, np.ndarray], out_dir: Path, cfg: SimConfig) -> None:
    t = data['t']

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    axes[0, 0].plot(t, data['capture_area_xy'], lw=2, label='围捕面积（XY）')
    axes[0, 0].plot(t, data['net_sag'], lw=1.8, label='网面下垂深度')
    axes[0, 0].axvline(cfg.command_time, color='r', ls='--', label='指令时刻')
    axes[0, 0].set_title('(a)可变形网面几何演化')
    axes[0, 0].set_xlabel('时间（秒）')
    axes[0, 0].set_ylabel('面积 / 下垂')
    axes[0, 0].legend()
    axes[0, 0].grid(alpha=0.3)

    for i in range(3):
        axes[0, 1].plot(t, data['rel_err'][:, i], label=f'跟随者{i+1}')
    axes[0, 1].axvline(cfg.command_time, color='r', ls='--')
    axes[0, 1].set_title('(b)跟随者三维编队跟踪误差')
    axes[0, 1].set_xlabel('时间（秒）')
    axes[0, 1].set_ylabel('||e_rel||（米）')
    axes[0, 1].legend()
    axes[0, 1].grid(alpha=0.3)

    for i in range(3):
        axes[1, 0].plot(t, data['est_err'][:, i], label=f'跟随者{i+1}')
    axes[1, 0].axvline(cfg.command_time, color='r', ls='--')
    axes[1, 0].set_title('(c)Leader 广播状态估计误差（三维）')
    axes[1, 0].set_xlabel('时间（秒）')
    axes[1, 0].set_ylabel('||状态估计 - Leader状态||（米）')
    axes[1, 0].legend()
    axes[1, 0].grid(alpha=0.3)

    axes[1, 1].plot(t, data['estimate_var'], color='tab:purple', lw=2, label='估计一致性方差')
    axes[1, 1].plot(t, data['surface_distance'], color='tab:red', lw=1.6, label='敌机到网面距离')
    axes[1, 1].plot(t, data['leader_enemy_dist'], color='tab:gray', lw=1.3, label='Leader-敌机距离')
    axes[1, 1].plot(t, data['downwash_intensity'], color='tab:cyan', lw=1.2, label='平均下洗扰动强度')
    axes[1, 1].axvline(cfg.command_time, color='r', ls='--')
    axes[1, 1].set_title('(d)一致性 + 围捕距离变化')
    axes[1, 1].set_xlabel('时间（秒）')
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
    labels = ['Leader', '跟随者-1', '跟随者-2', '跟随者-3']

    for k in range(0, len(t), frame_skip):
        ax.clear()
        ax.set_xlim(-32, 32)
        ax.set_ylim(-25, 25)
        ax.set_zlim(3, 15)
        ax.set_xlabel('X 轴（米）')
        ax.set_ylabel('Y 轴（米）')
        ax.set_zlabel('Z 轴（米）')
        ax.view_init(elev=26, azim=35)
        ax.set_title('四机 Leader/Follower 三维网捕仿真')

        state_txt = '密集巡航编队' if t[k] < cfg.command_time else '扩张后主动围捕'
        ax.text2D(0.02, 0.95, f'时间={t[k]:.1f}s | {state_txt}', transform=ax.transAxes)

        for i in range(4):
            ax.plot(x[:k + 1, i, 0], x[:k + 1, i, 1], x[:k + 1, i, 2], lw=1.2, color=colors[i])
            ax.scatter(x[k, i, 0], x[k, i, 1], x[k, i, 2], s=55, color=colors[i], label=labels[i])

        Xs, Ys, Zs, ordered_corners = build_net_surface(x[k], sag=data['net_sag'][k], res=17)
        ax.plot_wireframe(Xs, Ys, Zs, color='k', linewidth=0.7, rstride=2, cstride=2, alpha=0.8)

        # border of the net surface
        border = np.vstack([ordered_corners, ordered_corners[0]])
        ax.plot(border[:, 0], border[:, 1], border[:, 2], color='k', lw=2.0, alpha=0.9)

        ax.plot(enemy[:k + 1, 0], enemy[:k + 1, 1], enemy[:k + 1, 2], 'm--', lw=1.3, alpha=0.9)
        ax.scatter(enemy[k, 0], enemy[k, 1], enemy[k, 2], s=80, color='magenta', marker='x', label='敌方无人机')

        inside_xy = point_in_convex_quad_xy(enemy[k, :2], ordered_corners[:, :2])
        if inside_xy and data['surface_distance'][k] <= cfg.capture_surface_threshold:
            ax.text2D(0.02, 0.90, '敌机接触可变形网面', color='green', transform=ax.transAxes)
        if capture_start is not None and t[k] >= capture_start:
            ax.text2D(0.02, 0.86, f'捕获判定成立 @ {capture_start:.1f}s', color='green', transform=ax.transAxes)

        ax.text2D(
            0.02,
            0.82,
            f'网面距离={data["surface_distance"][k]:.2f} 米 | Leader-敌机={data["leader_enemy_dist"][k]:.2f} 米',
            transform=ax.transAxes,
        )
        ax.text2D(
            0.02,
            0.78,
            f'平均下洗扰动={data["downwash_intensity"][k]:.2f} m/s',
            transform=ax.transAxes,
        )

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
    parser.add_argument('--wide_tilt_deg', type=float, default=55.0, help='Target net tilt angle after expansion.')
    parser.add_argument('--downwash_strength', type=float, default=1.8, help='Downwash intensity scale (m/s).')
    parser.add_argument('--downwash_floor_ratio', type=float, default=0.28, help='Persistent downwash ratio after expansion.')
    parser.add_argument('--downwash_floor_xy', type=float, default=0.12, help='Persistent lateral wake amplitude after expansion (m/s).')
    parser.add_argument('--disable_downwash', action='store_true', help='Disable rotor downwash coupling model.')
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()

    out_dir = args.output_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    cfg = SimConfig(
        total_time=args.total_time,
        command_time=args.command_time,
        wide_tilt_deg=args.wide_tilt_deg,
        enable_downwash=not args.disable_downwash,
        downwash_strength=args.downwash_strength,
        downwash_floor_ratio=args.downwash_floor_ratio,
        downwash_floor_xy=args.downwash_floor_xy,
        seed=args.seed,
    )
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
