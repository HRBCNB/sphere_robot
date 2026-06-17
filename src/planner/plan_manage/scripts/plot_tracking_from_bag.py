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
                    msg.position.x, msg.position.y, msg.position.z,
                    msg.velocity.x, msg.velocity.y, msg.velocity.z,
                ])
            elif topic == odom_topic:
                odom_rows.append([
                    stamp,
                    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
                    msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
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
    odom_interp = np.zeros((ref.shape[0], 6), dtype=float)
    for col in range(1, 7):
        odom_interp[:, col - 1] = np.interp(t, odom[:, 0], odom[:, col])

    time = t - t[0]
    ref_state = ref[:, 1:7]
    return time, ref_state, odom_interp


def crop_time(time, ref_state, odom_state, start_time, end_time):
    mask = time >= start_time
    if end_time is not None:
        mask &= time <= end_time
    return time[mask], ref_state[mask], odom_state[mask]


def speed_norm(state):
    return np.linalg.norm(state[:, 3:6], axis=1)



def plot_spatial_path(path, ref_state, odom_state):
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.0))

    axes[0].plot(ref_state[:, 0], ref_state[:, 1], color='#1f77b4', linewidth=2.2, label='ref path')
    axes[0].plot(odom_state[:, 0], odom_state[:, 1], color='#d62728', linewidth=1.8, linestyle='--', label='odom path')
    axes[0].set_xlabel('x / m')
    axes[0].set_ylabel('y / m')
    axes[0].set_title('XY Path')
    axes[0].axis('equal')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='best')

    axes[1].plot(ref_state[:, 0], ref_state[:, 2], color='#1f77b4', linewidth=2.2, label='ref path')
    axes[1].plot(odom_state[:, 0], odom_state[:, 2], color='#d62728', linewidth=1.8, linestyle='--', label='odom path')
    axes[1].set_xlabel('x / m')
    axes[1].set_ylabel('z / m')
    axes[1].set_title('XZ Path / Jump Height')
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc='best')

    fig.suptitle('Spatial Tracking Path')
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(path, dpi=180)
    plt.close(fig)


def print_ranges(ref_state, odom_state):
    names = ['x', 'y', 'z']
    for i, name in enumerate(names):
        ref_min, ref_max = np.min(ref_state[:, i]), np.max(ref_state[:, i])
        odom_min, odom_max = np.min(odom_state[:, i]), np.max(odom_state[:, i])
        print('{} range: ref [{:.3f}, {:.3f}] span={:.3f}, odom [{:.3f}, {:.3f}] span={:.3f}'.format(
            name, ref_min, ref_max, ref_max - ref_min, odom_min, odom_max, odom_max - odom_min))

def save_csv(path, time, ref_state, odom_state):
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            't',
            'x_ref', 'y_ref', 'z_ref', 'vx_ref', 'vy_ref', 'vz_ref', 'v_ref_norm',
            'x_odom', 'y_odom', 'z_odom', 'vx_odom', 'vy_odom', 'vz_odom', 'v_odom_norm',
        ])
        ref_v = speed_norm(ref_state)
        odom_v = speed_norm(odom_state)
        for i in range(len(time)):
            writer.writerow([time[i], *ref_state[i], ref_v[i], *odom_state[i], odom_v[i]])


def plot_position(path, time, ref_state, odom_state):
    labels = [('x', 0), ('y', 1), ('z', 2)]
    fig, axes = plt.subplots(3, 1, figsize=(10, 7.2), sharex=True)
    for ax, (name, idx) in zip(axes, labels):
        ax.plot(time, ref_state[:, idx], color='#1f77b4', linewidth=2.0, label='{}_ref'.format(name))
        ax.plot(time, odom_state[:, idx], color='#d62728', linewidth=1.8, linestyle='--', label='{}_odom'.format(name))
        ax.set_ylabel('{} / m'.format(name))
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')
    axes[-1].set_xlabel('time / s')
    fig.suptitle('Reference Position vs Odometry Position')
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(path, dpi=180)
    plt.close(fig)



def plot_reference_velocity_axes(path, time, ref_state):
    labels = [('vx_ref', 3), ('vy_ref', 4), ('vz_ref', 5)]
    colors = ['#1f77b4', '#2ca02c', '#d62728']
    fig, axes = plt.subplots(3, 1, figsize=(10, 7.2), sharex=True)
    for ax, (label, idx), color in zip(axes, labels, colors):
        ax.plot(time, ref_state[:, idx], color=color, linewidth=2.0, label=label)
        ax.set_ylabel(label + ' / m/s')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')
    axes[-1].set_xlabel('time / s')
    fig.suptitle('Reference Velocity Components')
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(path, dpi=180)
    plt.close(fig)

def plot_speed(path, time, ref_state, odom_state):
    ref_v = speed_norm(ref_state)
    odom_v = speed_norm(odom_state)
    fig, ax = plt.subplots(figsize=(10, 4.6))
    ax.plot(time, ref_v, color='#1f77b4', linewidth=2.0, label='|v_ref|')
    ax.plot(time, odom_v, color='#d62728', linewidth=1.8, linestyle='--', label='|v_odom|')
    ax.set_xlabel('time / s')
    ax.set_ylabel('speed / m/s')
    ax.set_title('Reference Speed vs Odometry Speed')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description='Plot /planning/pos_cmd and /visual_slam/odom tracking curves from a rosbag.')
    parser.add_argument('bag', help='Input rosbag path')
    parser.add_argument('--pos-cmd-topic', default='/planning/pos_cmd')
    parser.add_argument('--odom-topic', default='/visual_slam/odom')
    parser.add_argument('--out-dir', default='tracking_plots')
    parser.add_argument('--start-time', type=float, default=0.0, help='Crop start time after alignment, seconds')
    parser.add_argument('--end-time', type=float, default=None, help='Crop end time after alignment, seconds')
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    ref, odom = read_bag(args.bag, args.pos_cmd_topic, args.odom_topic)
    time, ref_state, odom_state = align_to_ref(ref, odom)
    time, ref_state, odom_state = crop_time(time, ref_state, odom_state, args.start_time, args.end_time)

    if len(time) < 2:
        raise RuntimeError('Too little data after cropping')

    pos_png = out_dir / 'tracking_position.png'
    speed_png = out_dir / 'tracking_speed.png'
    ref_vel_axes_png = out_dir / 'reference_velocity_xyz.png'
    path_png = out_dir / 'tracking_path_xy_xz.png'
    csv_path = out_dir / 'tracking_aligned.csv'

    plot_position(pos_png, time, ref_state, odom_state)
    plot_speed(speed_png, time, ref_state, odom_state)
    plot_reference_velocity_axes(ref_vel_axes_png, time, ref_state)
    plot_spatial_path(path_png, ref_state, odom_state)
    save_csv(csv_path, time, ref_state, odom_state)

    pos_err = np.linalg.norm(ref_state[:, 0:3] - odom_state[:, 0:3], axis=1)
    vel_err = np.linalg.norm(ref_state[:, 3:6] - odom_state[:, 3:6], axis=1)
    print('Saved:')
    print('  {}'.format(pos_png))
    print('  {}'.format(speed_png))
    print('  {}'.format(ref_vel_axes_png))
    print('  {}'.format(path_png))
    print('  {}'.format(csv_path))
    print_ranges(ref_state, odom_state)
    print('Position error: mean={:.3f} m, max={:.3f} m'.format(np.mean(pos_err), np.max(pos_err)))
    print('Velocity error: mean={:.3f} m/s, max={:.3f} m/s, rms={:.3f} m/s'.format(
        np.mean(vel_err), np.max(vel_err), math.sqrt(np.mean(vel_err ** 2))))


if __name__ == '__main__':
    main()
