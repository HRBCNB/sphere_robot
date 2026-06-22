#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag


def msg_time(msg, bag_time):
    if hasattr(msg, 'header') and msg.header.stamp.to_sec() > 1e-9:
        return msg.header.stamp.to_sec()
    return bag_time.to_sec()


def read_bag(bag_path, pos_cmd_topic, odom_topic):
    ref_rows = []
    odom_rows = []
    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[pos_cmd_topic, odom_topic]):
            stamp = msg_time(msg, t)
            if topic == pos_cmd_topic:
                ref_rows.append([
                    stamp,
                    msg.position.x, msg.position.y,
                    msg.velocity.x, msg.velocity.y,
                    msg.acceleration.x, msg.acceleration.y,
                ])
            elif topic == odom_topic:
                odom_rows.append([
                    stamp,
                    msg.pose.pose.position.x, msg.pose.pose.position.y,
                    msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                ])
    if not ref_rows:
        raise RuntimeError('No messages found on {}'.format(pos_cmd_topic))
    if not odom_rows:
        raise RuntimeError('No messages found on {}'.format(odom_topic))
    return np.asarray(ref_rows, dtype=float), np.asarray(odom_rows, dtype=float)


def unique_by_time(data):
    order = np.argsort(data[:, 0])
    data = data[order]
    _, unique_idx = np.unique(data[:, 0], return_index=True)
    return data[np.sort(unique_idx)]


def align_to_ref(ref, odom):
    ref = unique_by_time(ref)
    odom = unique_by_time(odom)
    t0 = max(ref[0, 0], odom[0, 0])
    t1 = min(ref[-1, 0], odom[-1, 0])
    mask = (ref[:, 0] >= t0) & (ref[:, 0] <= t1)
    ref = ref[mask]
    if ref.shape[0] < 2:
        raise RuntimeError('Too little overlapping data between reference and odometry')
    t = ref[:, 0]
    odom_interp = np.zeros((ref.shape[0], 4), dtype=float)
    for col in range(1, 5):
        odom_interp[:, col - 1] = np.interp(t, odom[:, 0], odom[:, col])
    return t - t[0], ref[:, 1:7], odom_interp


def crop_time(time, ref_state, odom_state, start_time, end_time):
    mask = time >= start_time
    if end_time is not None:
        mask &= time <= end_time
    return time[mask], ref_state[mask], odom_state[mask]


def velocity_magnitude_xy(state):
    return np.linalg.norm(state[:, 2:4], axis=1)


def plot_position_xy(path, time, ref_state, odom_state):
    labels = [('x', 0), ('y', 1)]
    fig, axes = plt.subplots(2, 1, figsize=(10, 5.6), sharex=True)
    for ax, (name, idx) in zip(axes, labels):
        ax.plot(time, ref_state[:, idx], color='#1f77b4', linewidth=2.0, label='Reference position')
        ax.plot(time, odom_state[:, idx], color='#d62728', linewidth=1.8, linestyle='--', label='Actual position')
        ax.set_ylabel('{} position / m'.format(name))
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')
    axes[-1].set_xlabel('Time / s')
    fig.suptitle('XY Position Tracking')
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_velocity_components_xy(path, time, ref_state, odom_state):
    labels = [('x', 2), ('y', 3)]
    colors = ['#1f77b4', '#2ca02c']
    fig, axes = plt.subplots(2, 1, figsize=(10, 5.6), sharex=True)
    for ax, (name, idx), color in zip(axes, labels, colors):
        ax.plot(time, ref_state[:, idx], color=color, linewidth=2.0, label='Reference velocity')
        ax.plot(time, odom_state[:, idx], color='#d62728', linewidth=1.6, linestyle='--', label='Actual velocity')
        ax.set_ylabel('{} velocity / (m/s)'.format(name))
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')
    axes[-1].set_xlabel('Time / s')
    fig.suptitle('XY Velocity Component Tracking')
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_velocity_magnitude_xy(path, time, ref_state, odom_state):
    ref_v = velocity_magnitude_xy(ref_state)
    odom_v = velocity_magnitude_xy(odom_state)
    fig, ax = plt.subplots(figsize=(10, 4.6))
    ax.plot(time, ref_v, color='#1f77b4', linewidth=2.0, label='Reference velocity magnitude')
    ax.plot(time, odom_v, color='#d62728', linewidth=1.8, linestyle='--', label='Actual velocity magnitude')
    ax.set_xlabel('Time / s')
    ax.set_ylabel('Velocity magnitude / (m/s)')
    ax.set_title('XY Velocity Magnitude Tracking')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_path_xy(path, ref_state, odom_state):
    fig, ax = plt.subplots(figsize=(7.0, 6.0))
    ax.plot(ref_state[:, 0], ref_state[:, 1], color='#1f77b4', linewidth=2.2, label='Reference trajectory')
    ax.plot(odom_state[:, 0], odom_state[:, 1], color='#d62728', linewidth=1.8, linestyle='--', label='Actual trajectory')
    ax.set_xlabel('x position / m')
    ax.set_ylabel('y position / m')
    ax.set_title('XY Trajectory Tracking')
    ax.axis('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_position_error_xy(path, time, ref_state, odom_state):
    ex = ref_state[:, 0] - odom_state[:, 0]
    ey = ref_state[:, 1] - odom_state[:, 1]
    err = np.linalg.norm(ref_state[:, 0:2] - odom_state[:, 0:2], axis=1)
    max_idx = int(np.argmax(err))

    fig, axes = plt.subplots(2, 1, figsize=(10, 5.6), sharex=True)
    axes[0].plot(time, ex, color='#1f77b4', linewidth=1.8, label='$e_x$')
    axes[0].plot(time, ey, color='#2ca02c', linewidth=1.8, label='$e_y$')
    axes[0].set_ylabel('Position error / m')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='best')

    axes[1].plot(time, err, color='#d62728', linewidth=2.0, label='XY position error')
    axes[1].scatter(time[max_idx], err[max_idx], color='#111111', s=28, zorder=3)
    axes[1].annotate(
        'Max = {:.3f} m'.format(err[max_idx]),
        xy=(time[max_idx], err[max_idx]),
        xytext=(8, 10),
        textcoords='offset points',
    )
    axes[1].set_xlabel('Time / s')
    axes[1].set_ylabel('Error norm / m')
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc='best')

    fig.suptitle('XY Position Tracking Error')
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_reference_acceleration_xy(path, time, ref_state):
    ax_ref = ref_state[:, 4]
    ay_ref = ref_state[:, 5]
    acc_ref = np.linalg.norm(ref_state[:, 4:6], axis=1)
    max_idx = int(np.argmax(acc_ref))

    fig, axes = plt.subplots(2, 1, figsize=(10, 5.6), sharex=True)
    axes[0].plot(time, ax_ref, color='#1f77b4', linewidth=1.8, label='Reference $a_x$')
    axes[0].plot(time, ay_ref, color='#2ca02c', linewidth=1.8, label='Reference $a_y$')
    axes[0].set_ylabel('Acceleration / (m/s$^2$)')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='best')

    axes[1].plot(time, acc_ref, color='#d62728', linewidth=2.0, label='Reference acceleration magnitude')
    axes[1].scatter(time[max_idx], acc_ref[max_idx], color='#111111', s=28, zorder=3)
    axes[1].annotate(
        'Max = {:.3f} m/s$^2$'.format(acc_ref[max_idx]),
        xy=(time[max_idx], acc_ref[max_idx]),
        xytext=(8, 10),
        textcoords='offset points',
    )
    axes[1].set_xlabel('Time / s')
    axes[1].set_ylabel('Acceleration magnitude / (m/s$^2$)')
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc='best')

    fig.suptitle('Reference Trajectory Acceleration')
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(path, dpi=180)
    plt.close(fig)


def save_csv(path, time, ref_state, odom_state):
    ref_v = velocity_magnitude_xy(ref_state)
    ref_a = np.linalg.norm(ref_state[:, 4:6], axis=1)
    odom_v = velocity_magnitude_xy(odom_state)
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            't',
            'x_ref', 'y_ref', 'vx_ref', 'vy_ref', 'v_xy_ref',
            'ax_ref', 'ay_ref', 'a_xy_ref',
            'x_odom', 'y_odom', 'vx_odom', 'vy_odom', 'v_xy_odom',
        ])
        for i in range(len(time)):
            writer.writerow([
                time[i],
                *ref_state[i, 0:4], ref_v[i],
                *ref_state[i, 4:6], ref_a[i],
                *odom_state[i], odom_v[i],
            ])


def print_ranges(ref_state, odom_state):
    for i, name in enumerate(['x', 'y']):
        ref_min, ref_max = np.min(ref_state[:, i]), np.max(ref_state[:, i])
        odom_min, odom_max = np.min(odom_state[:, i]), np.max(odom_state[:, i])
        print('{} range: ref [{:.3f}, {:.3f}] span={:.3f}, odom [{:.3f}, {:.3f}] span={:.3f}'.format(
            name, ref_min, ref_max, ref_max - ref_min, odom_min, odom_max, odom_max - odom_min))


def main():
    parser = argparse.ArgumentParser(description='Plot ROLL-only XY tracking curves from a rosbag. Z axis is omitted.')
    parser.add_argument('bag', help='Input rosbag path')
    parser.add_argument('--pos-cmd-topic', default='/planning/pos_cmd')
    parser.add_argument('--odom-topic', default='/visual_slam/odom')
    parser.add_argument('--out-dir', default='tracking_plots_roll')
    parser.add_argument('--start-time', type=float, default=0.0)
    parser.add_argument('--end-time', type=float, default=None)
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    ref, odom = read_bag(args.bag, args.pos_cmd_topic, args.odom_topic)
    time, ref_state, odom_state = align_to_ref(ref, odom)
    time, ref_state, odom_state = crop_time(time, ref_state, odom_state, args.start_time, args.end_time)
    if len(time) < 2:
        raise RuntimeError('Too little data after cropping')

    pos_png = out_dir / '位置跟踪曲线.png'
    vel_components_png = out_dir / '速度分量跟踪曲线.png'
    vel_magnitude_png = out_dir / '速度大小跟踪曲线.png'
    path_png = out_dir / '平面轨迹对比.png'
    pos_err_png = out_dir / '位置跟踪误差曲线.png'
    ref_acc_png = out_dir / '参考加速度曲线.png'
    csv_path = out_dir / 'roll_tracking_aligned_xy.csv'

    plot_position_xy(pos_png, time, ref_state, odom_state)
    plot_velocity_components_xy(vel_components_png, time, ref_state, odom_state)
    plot_velocity_magnitude_xy(vel_magnitude_png, time, ref_state, odom_state)
    plot_path_xy(path_png, ref_state, odom_state)
    plot_position_error_xy(pos_err_png, time, ref_state, odom_state)
    plot_reference_acceleration_xy(ref_acc_png, time, ref_state)
    save_csv(csv_path, time, ref_state, odom_state)

    pos_err = np.linalg.norm(ref_state[:, 0:2] - odom_state[:, 0:2], axis=1)
    vel_err = np.linalg.norm(ref_state[:, 2:4] - odom_state[:, 2:4], axis=1)
    ref_acc = np.linalg.norm(ref_state[:, 4:6], axis=1)

    print('Saved:')
    for p in [pos_png, vel_components_png, vel_magnitude_png, path_png, pos_err_png, ref_acc_png, csv_path]:
        print('  {}'.format(p))
    print_ranges(ref_state, odom_state)
    print('XY position error: mean={:.3f} m, max={:.3f} m'.format(np.mean(pos_err), np.max(pos_err)))
    print('XY velocity error: mean={:.3f} m/s, max={:.3f} m/s, rms={:.3f} m/s'.format(
        np.mean(vel_err), np.max(vel_err), math.sqrt(np.mean(vel_err ** 2))))
    print('XY reference acceleration: mean={:.3f} m/s^2, max={:.3f} m/s^2'.format(
        np.mean(ref_acc), np.max(ref_acc)))


if __name__ == '__main__':
    main()
