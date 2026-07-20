#!/usr/bin/env python3
"""
偏航角跟踪对比 — 缩放至动态变化最明显的窗口，突出相位偏差
"""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pyulog import ULog

ULG_FILE = os.path.expanduser(
    "~/PX4-Autopilot-canard-2026.6.2/build/px4_sitl_default/rootfs/log/2026-07-08/10_25_26.ulg"
)

plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


def quat_to_yaw(q):
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


ulog = ULog(ULG_FILE)

# --- 数据提取 ---
att = ulog.get_dataset('vehicle_attitude')
t0 = att.data['timestamp'][0] / 1e6
t_att = att.data['timestamp'] / 1e6 - t0
q_act = np.column_stack([att.data['q[0]'], att.data['q[1]'],
                          att.data['q[2]'], att.data['q[3]']])
yaw_deg = np.degrees(quat_to_yaw(q_act))

att_sp = ulog.get_dataset('vehicle_attitude_setpoint')
t_att_sp = att_sp.data['timestamp'] / 1e6 - t0
q_sp = np.column_stack([att_sp.data['q_d[0]'], att_sp.data['q_d[1]'],
                         att_sp.data['q_d[2]'], att_sp.data['q_d[3]']])
yaw_sp_deg = np.degrees(quat_to_yaw(q_sp))

# --- 设定时间窗口: 取偏航动态最活跃的前 80 秒 ---
T_START, T_END = 5, 85  # 秒

idx_act = (t_att >= T_START) & (t_att <= T_END)
mask_sp = ~np.isnan(yaw_sp_deg)
idx_sp = mask_sp & (t_att_sp >= T_START) & (t_att_sp <= T_END)

# --- 误差计算 ---
yaw_sp_interp_full = np.interp(t_att, t_att_sp[mask_sp], yaw_sp_deg[mask_sp])
yaw_err = ((yaw_deg - yaw_sp_interp_full) + 180.0) % 360.0 - 180.0
yaw_sp_aligned = yaw_deg - yaw_err
rmse = np.sqrt(np.mean(yaw_err[idx_act]**2))

# ================================================================
# 绘制 (线型与 analyze_yaw.py 子图 (3a) 一致，窗口聚焦动态变化区域)
# ================================================================
C_ACTUAL = '#2196F3'
C_SP = '#FF5722'

fig, ax = plt.subplots(figsize=(18, 5.5))

ax.plot(t_att_sp[idx_sp], yaw_sp_deg[idx_sp], color=C_SP, linewidth=1.2,
        linestyle='-.', label='Yaw Setpoint')
ax.plot(t_att[idx_act], yaw_deg[idx_act], color=C_ACTUAL, linewidth=1.0,
        label='Actual Yaw')
ax.fill_between(t_att[idx_act], yaw_deg[idx_act], yaw_sp_aligned[idx_act],
                alpha=0.12, color=C_SP)

ax.set_xlabel('Time [s]', fontsize=12)
ax.set_ylabel('Yaw Angle [deg]', fontsize=12)
ax.set_title(f'Yaw Tracking: Angle  (zoom {T_START}s–{T_END}s, phase lag visible)',
             fontsize=14, fontweight='bold')
ax.legend(loc='upper right', fontsize=11)
ax.grid(True, alpha=0.3)
ax.set_xlim(T_START, T_END)

# RMSE 标注
ax.text(0.02, 0.04, f'RMSE = {rmse:.2f}°', transform=ax.transAxes,
        fontsize=11, verticalalignment='bottom',
        bbox=dict(boxstyle='round,pad=0.4', facecolor='lightyellow', alpha=0.85))

output = os.path.expanduser('~/PX4-Autopilot-canard-2026.6.2/yaw_setpoint_vs_actual.png')
fig.savefig(output, dpi=150, bbox_inches='tight', facecolor='white')
print(f'图表已保存至: {output}')
print(f'RMSE ({T_START}s–{T_END}s): {rmse:.3f}°')
