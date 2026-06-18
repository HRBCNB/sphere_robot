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
    return t - t[0], ref[:, 1:5], odom_interp


def crop_time(time, ref_state, odom_state, start_time, end_time):
    mask = time >= start_time
    if end_time is not None:
        mask &= time <= end_time
    return time[mask], ref_state[mask], odom_state[mask]


def speed_xy(state):
    return np.linalg.norm(state[:, 2:4], axis=1)


def plot_position_xy(path, time, ref_state, odom_state):
    labels = [('x', 0), ('y', 1)]
    fig, axes = plt.subplots(2, 1, figsize=(10, 5.6), sharex=True)
    for ax, (name, idx) in zip(axes, labels):
        ax.plot(time, ref_state[:, idx], color='#1f77b4', linewidth=2.0, label='{}_ref'.format(name))
        ax.plot(time, odom_state[:, idx], color='#d62728', linewidth=1.8, linestyle='--', label='{}_odom'.format(name))
        ax.set_ylabel('{} / m'.format(name))
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')
    axes[-1].set_xlabel('time / s')
    fig.suptitle('ROLL Tracking Position')
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_velocity_xy(path, time, ref_state, odom_state):
    labels = [('vx', 2), ('vy', 3)]
    colors = ['#1f77b4', '#2ca02c']
    fig, axes = plt.subplots(2, 1, figsize=(10, 5.6), sharex=True)
    for ax, (name, idx), color in zip(axes, labels, colors):
        ax.plot(time, ref_state[:, idx], color=color, linewidth=2.0, label='{}_ref'.format(name))
        ax.plot(time, odom_state[:, idx], color='#d62728', linewidth=1.6, linestyle='--', label='{}_odom'.format(name))
        ax.set_ylabel('{} / m/s'.format(name))
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')
    axes[-1].set_xlabel('time / s')
    fig.suptitle('ROLL Tracking Velocity Components')
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_speed_xy(path, time, ref_state, odom_state):
    ref_v = speed_xy(ref_state)
    odom_v = speed_xy(odom_state)
    fig, ax = plt.subplots(figsize=(10, 4.6))
    ax.plot(time, ref_v, color='#1f77b4', linewidth=2.0, label='|v_xy_ref|')
    ax.plot(time, odom_v, color='#d62728', linewidth=1.8, linestyle='--', label='|v_xy_odom|')
    ax.set_xlabel('time / s')
    ax.set_ylabel('speed / m/s')
    ax.set_title('ROLL Tracking XY Speed')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_path_xy(path, ref_state, odom_state):
    fig, ax = plt.subplots(figsize=(7.0, 6.0))
    ax.plot(ref_state[:, 0], ref_state[:, 1], color='#1f77b4', linewidth=2.2, label='ref path')
    ax.plot(odom_state[:, 0], odom_state[:, 1], color='#d62728', linewidth=1.8, linestyle='--', label='odom path')
    ax.set_xlabel('x / m')
    ax.set_ylabel('y / m')
    ax.set_title('ROLL Tracking XY Path')
    ax.axis('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best')
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def save_csv(path, time, ref_state, odom_state):
    ref_v = speed_xy(ref_state)
    odom_v = speed_xy(odom_state)
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            't',
            'x_ref', 'y_ref', 'vx_ref', 'vy_ref', 'v_xy_ref',
            'x_odom', 'y_odom', 'vx_odom', 'vy_odom', 'v_xy_odom',
        ])
        for i in range(len(time)):
            writer.writerow([time[i], *ref_state[i], ref_v[i], *odom_state[i], odom_v[i]])


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

    pos_png = out_dir / 'roll_tracking_position_xy.png'
    vel_png = out_dir / 'roll_tracking_velocity_xy.png'
    speed_png = out_dir / 'roll_tracking_speed_xy.png'
    path_png = out_dir / 'roll_tracking_path_xy.png'
    csv_path = out_dir / 'roll_tracking_aligned_xy.csv'

    plot_position_xy(pos_png, time, ref_state, odom_state)
    plot_velocity_xy(vel_png, time, ref_state, odom_state)
    plot_speed_xy(speed_png, time, ref_state, odom_state)
    plot_path_xy(path_png, ref_state, odom_state)
    save_csv(csv_path, time, ref_state, odom_state)

    pos_err = np.linalg.norm(ref_state[:, 0:2] - odom_state[:, 0:2], axis=1)
    vel_err = np.linalg.norm(ref_state[:, 2:4] - odom_state[:, 2:4], axis=1)

    print('Saved:')
    for p in [pos_png, vel_png, speed_png, path_png, csv_path]:
        print('  {}'.format(p))
    print_ranges(ref_state, odom_state)
    print('XY position error: mean={:.3f} m, max={:.3f} m'.format(np.mean(pos_err), np.max(pos_err)))
    print('XY velocity error: mean={:.3f} m/s, max={:.3f} m/s, rms={:.3f} m/s'.format(
        np.mean(vel_err), np.max(vel_err), math.sqrt(np.mean(vel_err ** 2))))


if __name__ == '__main__':
    main()
