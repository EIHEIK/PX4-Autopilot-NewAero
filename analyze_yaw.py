#!/usr/bin/env python3
"""
PX4 飞行日志偏航 (Yaw) 综合分析脚本
=============================================
分析 .ulg 日志中的飞机偏航相关信息，包括:
  1. 偏航角 (Yaw Angle) — 从 vehicle_attitude 四元数转换
  2. 偏航角速度 (Yaw Rate) — vehicle_angular_velocity.xyz[2]
  3. 偏航角设定值 (Yaw Setpoint) — vehicle_attitude_setpoint / vehicle_rates_setpoint
  4. 偏航估计器状态 (Yaw Estimator) — 来自 GNSS/Mag/Vision 等多源 yaw 估计
  5. 偏航设定值与实际值对比
  6. 偏航角速度设定值与实际值对比
  7. 多源偏航估计对比 (GNSS/Mag 等)

参考:
  - PX4 uORB topics: vehicle_attitude, vehicle_angular_velocity,
    vehicle_attitude_setpoint, vehicle_rates_setpoint, yaw_estimator_status
  - 项目代码: src/modules/ekf2/EKF2.cpp (yaw_estimator_status)
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 无 GUI 后端
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
from pyulog import ULog
from pyulog.core import ULog
import matplotlib.gridspec as gridspec

# ============================================================
# 配置
# ============================================================
ULG_FILE = os.path.expanduser(
    "~/PX4-Autopilot-canard-2026.6.2/build/px4_sitl_default/rootfs/log/2026-07-08/10_25_26.ulg"
)
OUTPUT_DIR = os.path.expanduser("~/PX4-Autopilot-canard-2026.6.2")

# 中文字体配置
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei', 'WenQuanYi Micro Hei', 'Noto Sans CJK SC']
plt.rcParams['axes.unicode_minus'] = False


def quat_to_euler(q):
    """
    将四元数 [w, x, y, z] (PX4: q[0]=w, q[1]=x, q[2]=y, q[3]=z)
    转换为 Euler 角 [roll, pitch, yaw] (rad), 范围 [-pi, pi]
    """
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]

    # Roll
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch
    sinp = 2.0 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1.0,
                     np.sign(sinp) * np.pi / 2,
                     np.arcsin(sinp))

    # Yaw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def get_data(ulog, topic_name, multi_instance=0):
    """从 ULog 中提取指定 topic 的数据"""
    data = ulog.get_dataset(topic_name, multi_instance=multi_instance)
    if data is None:
        return None
    return data


def rad_to_deg(rad):
    """弧度转角度"""
    return rad * 180.0 / np.pi


def main():
    print("=" * 60)
    print("PX4 飞行日志偏航 (Yaw) 综合分析")
    print(f"日志文件: {ULG_FILE}")
    print("=" * 60)

    # 加载日志
    if not os.path.exists(ULG_FILE):
        print(f"错误: 日志文件不存在: {ULG_FILE}")
        sys.exit(1)

    ulog = ULog(ULG_FILE)
    print(f"日志时长: {ulog.get_dataset('vehicle_attitude').data['timestamp'][-1] / 1e6:.1f} 秒")

    # ================================================================
    # 1. 提取 vehicle_attitude → 偏航角 (实际)
    # ================================================================
    att = get_data(ulog, 'vehicle_attitude')
    t_att = att.data['timestamp'] / 1e6  # 转换为秒
    t_att -= t_att[0]  # 从 0 开始
    q = np.column_stack([
        att.data['q[0]'], att.data['q[1]'],
        att.data['q[2]'], att.data['q[3]']
    ])
    roll, pitch, yaw = quat_to_euler(q)
    yaw_deg = rad_to_deg(yaw)

    # ================================================================
    # 2. 提取 vehicle_attitude_groundtruth → 偏航角真值
    # ================================================================
    att_gt = get_data(ulog, 'vehicle_attitude_groundtruth')
    yaw_gt_deg = None
    t_att_gt = None
    if att_gt is not None:
        t_att_gt = att_gt.data['timestamp'] / 1e6 - t_att[0]  # 与 attitude 对齐时间
        q_gt = np.column_stack([
            att_gt.data['q[0]'], att_gt.data['q[1]'],
            att_gt.data['q[2]'], att_gt.data['q[3]']
        ])
        _, _, yaw_gt = quat_to_euler(q_gt)
        yaw_gt_deg = rad_to_deg(yaw_gt)
        print(f"  vehicle_attitude_groundtruth: {len(t_att_gt)} 数据点")

    # ================================================================
    # 3. 提取 vehicle_angular_velocity → 偏航角速度 (实际)
    # ================================================================
    angvel = get_data(ulog, 'vehicle_angular_velocity')
    t_av = angvel.data['timestamp'] / 1e6 - t_att[0]
    yaw_rate = rad_to_deg(angvel.data['xyz[2]'])  # Z 轴 = yaw rate, deg/s

    # ================================================================
    # 4. 提取 vehicle_angular_velocity_groundtruth → 偏航角速度真值
    # ================================================================
    angvel_gt = get_data(ulog, 'vehicle_angular_velocity_groundtruth')
    yaw_rate_gt = None
    t_av_gt = None
    if angvel_gt is not None:
        t_av_gt = angvel_gt.data['timestamp'] / 1e6 - t_att[0]
        yaw_rate_gt = rad_to_deg(angvel_gt.data['xyz[2]'])
        print(f"  vehicle_angular_velocity_groundtruth: {len(t_av_gt)} 数据点")

    # ================================================================
    # 5. 提取 vehicle_attitude_setpoint → 偏航设定值
    # ================================================================
    att_sp = get_data(ulog, 'vehicle_attitude_setpoint')
    yaw_sp_deg = None
    t_att_sp = None
    yaw_sp_move_rate = None
    if att_sp is not None:
        t_att_sp = att_sp.data['timestamp'] / 1e6 - t_att[0]
        q_sp = np.column_stack([
            att_sp.data['q_d[0]'], att_sp.data['q_d[1]'],
            att_sp.data['q_d[2]'], att_sp.data['q_d[3]']
        ])
        _, _, yaw_sp = quat_to_euler(q_sp)
        # 处理 NaN setpoint
        yaw_sp_deg = rad_to_deg(yaw_sp)
        yaw_sp_move_rate = rad_to_deg(att_sp.data['yaw_sp_move_rate'])
        print(f"  vehicle_attitude_setpoint: {len(t_att_sp)} 数据点")

    # ================================================================
    # 6. 提取 vehicle_rates_setpoint → 偏航角速度设定值
    # ================================================================
    rates_sp = get_data(ulog, 'vehicle_rates_setpoint')
    yaw_rate_sp = None
    t_rates_sp = None
    if rates_sp is not None:
        t_rates_sp = rates_sp.data['timestamp'] / 1e6 - t_att[0]
        yaw_rate_sp = rad_to_deg(rates_sp.data['yaw'])
        print(f"  vehicle_rates_setpoint: {len(t_rates_sp)} 数据点")

    # ================================================================
    # 7. 提取 yaw_estimator_status → 多源偏航估计
    # ================================================================
    yaw_est = get_data(ulog, 'yaw_estimator_status')
    t_yaw_est = None
    yaw_sources = None
    yaw_composite = None
    yaw_composite_valid = None
    if yaw_est is not None:
        t_yaw_est = yaw_est.data['timestamp'] / 1e6 - t_att[0]
        n_sources = 5  # yaw[0..4]
        yaw_sources = np.column_stack([
            yaw_est.data[f'yaw[{i}]'] for i in range(n_sources)
        ])
        yaw_sources_deg = rad_to_deg(yaw_sources)
        yaw_composite = rad_to_deg(yaw_est.data['yaw_composite'])
        yaw_composite_valid = yaw_est.data['yaw_composite_valid']

        # 权重
        weights = np.column_stack([
            yaw_est.data[f'weight[{i}]'] for i in range(n_sources)
        ])

        # Innovation
        innov_vn = np.column_stack([
            yaw_est.data[f'innov_vn[{i}]'] for i in range(n_sources)
        ])
        innov_ve = np.column_stack([
            yaw_est.data[f'innov_ve[{i}]'] for i in range(n_sources)
        ])

        print(f"  yaw_estimator_status: {len(t_yaw_est)} 数据点, {n_sources} 个估计源")

    # ================================================================
    # 8. 提取 rate_ctrl_status → 偏航速率积分项
    # ================================================================
    rate_ctrl = get_data(ulog, 'rate_ctrl_status')
    t_rate_ctrl = None
    yawspeed_integ = None
    if rate_ctrl is not None:
        t_rate_ctrl = rate_ctrl.data['timestamp'] / 1e6 - t_att[0]
        yawspeed_integ = rate_ctrl.data['yawspeed_integ']
        print(f"  rate_ctrl_status: {len(t_rate_ctrl)} 数据点")

    print("\n数据提取完成，开始绘图...\n")

    # ================================================================
    # 绘图
    # ================================================================
    fig = plt.figure(figsize=(22, 24))
    gs = gridspec.GridSpec(7, 2, figure=fig,
                           hspace=0.4, wspace=0.3,
                           top=0.95, bottom=0.03,
                           left=0.06, right=0.97)

    # 颜色主题
    C_ACTUAL = '#2196F3'       # 蓝色 - 实际值
    C_GT = '#4CAF50'           # 绿色 - 真值
    C_SP = '#FF5722'           # 橙色 - 设定值
    C_EST = ['#E91E63', '#9C27B0', '#3F51B5', '#009688', '#FF9800']  # 估计源颜色
    ALPHA_FILL = 0.12

    # ---- Row 1: 偏航角对比 ----
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(t_att, yaw_deg, color=C_ACTUAL, linewidth=1.0, label='Yaw Actual (EKF estimated)')
    if yaw_gt_deg is not None:
        ax1.plot(t_att_gt, yaw_gt_deg, color=C_GT, linewidth=0.8, linestyle='--',
                 label='Yaw Ground Truth')
    if yaw_sp_deg is not None:
        # 过滤掉 NaN setpoint
        mask = ~np.isnan(yaw_sp_deg)
        ax1.plot(t_att_sp[mask], yaw_sp_deg[mask], color=C_SP, linewidth=1.2,
                 linestyle='-.', label='Yaw Setpoint')
    ax1.set_ylabel('Yaw Angle [deg]', fontsize=11)
    ax1.set_title('(1) Yaw Angle: Actual vs Ground Truth vs Setpoint', fontsize=13, fontweight='bold')
    ax1.legend(loc='upper right', fontsize=9, ncol=3)
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color='gray', linewidth=0.5, linestyle=':')

    # ---- Row 2: 偏航角速度对比 ----
    ax2 = fig.add_subplot(gs[1, :])
    ax2.plot(t_av, yaw_rate, color=C_ACTUAL, linewidth=1.0, label='Yaw Rate Actual')
    if yaw_rate_gt is not None:
        ax2.plot(t_av_gt, yaw_rate_gt, color=C_GT, linewidth=0.8, linestyle='--',
                 label='Yaw Rate Ground Truth')
    if yaw_rate_sp is not None:
        ax2.plot(t_rates_sp, yaw_rate_sp, color=C_SP, linewidth=1.2, linestyle='-.',
                 label='Yaw Rate Setpoint')
    ax2.set_ylabel('Yaw Rate [deg/s]', fontsize=11)
    ax2.set_title('(2) Yaw Rate: Actual vs Ground Truth vs Setpoint', fontsize=13, fontweight='bold')
    ax2.legend(loc='upper right', fontsize=9, ncol=3)
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='gray', linewidth=0.5, linestyle=':')

    # ---- Row 3: 偏航设定值 vs 实际值 (重叠对比) ----
    ax3_l = fig.add_subplot(gs[2, 0])
    ax3_l.plot(t_att, yaw_deg, color=C_ACTUAL, linewidth=1.0, label='Actual Yaw')
    if yaw_sp_deg is not None:
        mask = ~np.isnan(yaw_sp_deg)
        ax3_l.plot(t_att_sp[mask], yaw_sp_deg[mask], color=C_SP, linewidth=1.2,
                   linestyle='-.', label='Yaw Setpoint')
    ax3_l.set_ylabel('Yaw Angle [deg]', fontsize=11)
    ax3_l.set_title('(3a) Yaw Tracking: Angle', fontsize=12, fontweight='bold')
    ax3_l.legend(loc='upper right', fontsize=8)
    ax3_l.grid(True, alpha=0.3)

    ax3_r = fig.add_subplot(gs[2, 1])
    ax3_r.plot(t_av, yaw_rate, color=C_ACTUAL, linewidth=1.0, label='Actual Yaw Rate')
    if yaw_rate_sp is not None:
        ax3_r.plot(t_rates_sp, yaw_rate_sp, color=C_SP, linewidth=1.2, linestyle='-.',
                   label='Yaw Rate Setpoint')
    ax3_r.set_ylabel('Yaw Rate [deg/s]', fontsize=11)
    ax3_r.set_title('(3b) Yaw Tracking: Rate', fontsize=12, fontweight='bold')
    ax3_r.legend(loc='upper right', fontsize=8)
    ax3_r.grid(True, alpha=0.3)

    # ---- Row 4: 偏航误差 ----
    ax4_l = fig.add_subplot(gs[3, 0])
    if yaw_sp_deg is not None:
        # 将 setpoint 插值到 attitude 时间轴
        mask_sp = ~np.isnan(yaw_sp_deg)
        yaw_sp_interp = np.interp(t_att, t_att_sp[mask_sp], yaw_sp_deg[mask_sp])
        yaw_error = yaw_deg - yaw_sp_interp
        ax4_l.fill_between(t_att, yaw_error, 0, alpha=ALPHA_FILL, color=C_SP)
        ax4_l.plot(t_att, yaw_error, color=C_SP, linewidth=1.0)
    ax4_l.set_ylabel('Yaw Error [deg]', fontsize=11)
    ax4_l.set_title('(4a) Yaw Angle Error (Actual - Setpoint)', fontsize=12, fontweight='bold')
    ax4_l.grid(True, alpha=0.3)
    ax4_l.axhline(y=0, color='gray', linewidth=0.5, linestyle=':')

    ax4_r = fig.add_subplot(gs[3, 1])
    if yaw_rate_sp is not None:
        yaw_rate_sp_interp = np.interp(t_av, t_rates_sp, yaw_rate_sp)
        yaw_rate_error = yaw_rate - yaw_rate_sp_interp
        ax4_r.fill_between(t_av, yaw_rate_error, 0, alpha=ALPHA_FILL, color=C_SP)
        ax4_r.plot(t_av, yaw_rate_error, color=C_SP, linewidth=1.0)
    ax4_r.set_ylabel('Yaw Rate Error [deg/s]', fontsize=11)
    ax4_r.set_title('(4b) Yaw Rate Error (Actual - Setpoint)', fontsize=12, fontweight='bold')
    ax4_r.grid(True, alpha=0.3)
    ax4_r.axhline(y=0, color='gray', linewidth=0.5, linestyle=':')

    # ---- Row 5: 多源偏航估计对比 ----
    ax5 = fig.add_subplot(gs[4, :])
    if yaw_sources is not None:
        source_labels = ['GNSS Yaw', 'Mag Yaw (0)', 'Mag Yaw (1)', 'Vision Yaw', 'Mag Yaw (2)']
        for i in range(min(5, yaw_sources.shape[1])):
            valid_mask = np.abs(yaw_sources_deg[:, i]) < 1e4  # 过滤无效值
            ax5.plot(t_yaw_est[valid_mask], yaw_sources_deg[valid_mask, i],
                     color=C_EST[i], linewidth=1.0, alpha=0.7,
                     label=f'{source_labels[i]}')
        ax5.plot(t_yaw_est, yaw_composite, color='black', linewidth=1.8,
                 linestyle='-', label='Composite Yaw')
        # 同时画实际偏航
        ax5.plot(t_att, yaw_deg, color=C_ACTUAL, linewidth=1.2, linestyle='--',
                 alpha=0.6, label='EKF Yaw (vehicle_attitude)')
    ax5.set_ylabel('Yaw Angle [deg]', fontsize=11)
    ax5.set_title('(5) Multi-Source Yaw Estimates (yaw_estimator_status)', fontsize=13, fontweight='bold')
    ax5.legend(loc='upper right', fontsize=8, ncol=4)
    ax5.grid(True, alpha=0.3)

    # ---- Row 6: 偏航估计器权重 & 创新 ----
    ax6_l = fig.add_subplot(gs[5, 0])
    if weights is not None:
        source_labels = ['GNSS', 'Mag0', 'Mag1', 'Vision', 'Mag2']
        for i in range(min(5, weights.shape[1])):
            ax6_l.plot(t_yaw_est, weights[:, i], color=C_EST[i], linewidth=1.0,
                       label=source_labels[i], alpha=0.8)
    ax6_l.set_ylabel('Weight', fontsize=11)
    ax6_l.set_title('(6a) Yaw Estimator Weights', fontsize=12, fontweight='bold')
    ax6_l.legend(loc='upper right', fontsize=8, ncol=5)
    ax6_l.set_ylim(-0.05, 1.2)
    ax6_l.grid(True, alpha=0.3)

    ax6_r = fig.add_subplot(gs[5, 1])
    if yaw_sources is not None:
        innov_total = np.sqrt(innov_vn**2 + innov_ve**2)
        for i in range(min(5, innov_total.shape[1])):
            ax6_r.plot(t_yaw_est, innov_total[:, i], color=C_EST[i], linewidth=0.8,
                       label=f'{source_labels[i]}', alpha=0.8)
    ax6_r.set_ylabel('Innovation Norm [rad]', fontsize=11)
    ax6_r.set_title('(6b) Yaw Estimator Innovation (|VN|^2 + |VE|^2)^0.5', fontsize=12, fontweight='bold')
    ax6_r.legend(loc='upper right', fontsize=8, ncol=5)
    ax6_r.grid(True, alpha=0.3)

    # ---- Row 7: 偏航速率控制器积分项 ----
    ax7 = fig.add_subplot(gs[6, :])
    if yawspeed_integ is not None:
        ax7.plot(t_rate_ctrl, yawspeed_integ, color='#795548', linewidth=1.0,
                 label='Yawspeed Integral (rate_ctrl_status)')
    ax7.set_xlabel('Time [s]', fontsize=11)
    ax7.set_ylabel('Yawspeed Integral', fontsize=11)
    ax7.set_title('(7) Rate Controller Yawspeed Integral', fontsize=13, fontweight='bold')
    ax7.legend(loc='upper right', fontsize=9)
    ax7.grid(True, alpha=0.3)
    ax7.axhline(y=0, color='gray', linewidth=0.5, linestyle=':')

    # ---- 总体标题 ----
    fig.suptitle(f'PX4 Flight Log — Yaw Analysis\n{os.path.basename(ULG_FILE)}',
                 fontsize=16, fontweight='bold', y=0.98)

    # 保存
    output_png = os.path.join(OUTPUT_DIR, 'yaw_analysis.png')
    fig.savefig(output_png, dpi=150, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    print(f"\n图表已保存至: {output_png}")

    # ================================================================
    # 统计摘要
    # ================================================================
    print("\n" + "=" * 60)
    print("偏航数据统计摘要")
    print("=" * 60)

    print(f"\n偏航角 (Yaw Angle):")
    print(f"  范围: {yaw_deg.min():.2f}° ~ {yaw_deg.max():.2f}°")
    print(f"  均值: {yaw_deg.mean():.2f}°")
    print(f"  标准差: {yaw_deg.std():.2f}°")

    print(f"\n偏航角速度 (Yaw Rate):")
    print(f"  范围: {yaw_rate.min():.2f}°/s ~ {yaw_rate.max():.2f}°/s")
    print(f"  均值: {yaw_rate.mean():.2f}°/s")
    print(f"  标准差: {yaw_rate.std():.2f}°/s")

    if yaw_sp_deg is not None:
        mask = ~np.isnan(yaw_sp_deg)
        yaw_sp_interp = np.interp(t_att, t_att_sp[mask], yaw_sp_deg[mask])
        yaw_error = yaw_deg - yaw_sp_interp
        print(f"\n偏航角跟踪误差:")
        print(f"  RMSE: {np.sqrt(np.mean(yaw_error**2)):.3f}°")
        print(f"  最大误差: {np.abs(yaw_error).max():.3f}°")

    if yaw_rate_sp is not None:
        yaw_rate_sp_interp = np.interp(t_av, t_rates_sp, yaw_rate_sp)
        yaw_rate_error = yaw_rate - yaw_rate_sp_interp
        print(f"\n偏航角速度跟踪误差:")
        print(f"  RMSE: {np.sqrt(np.mean(yaw_rate_error**2)):.3f}°/s")
        print(f"  最大误差: {np.abs(yaw_rate_error).max():.3f}°/s")

    print("\n完成!")


if __name__ == '__main__':
    main()
