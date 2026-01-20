#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import threading
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import matplotlib.gridspec as gridspec
import sys
import os
import csv
import datetime
from collections import deque

# --- Dependency Check ---
try:
    from ros_interfaces.msg import MpcPositionCommand
except ImportError:
    print("Error: Could not import ros_interfaces.msg.MpcPositionCommand")
    sys.exit(1)

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

# --- Configuration ---
TOPIC_OPT_PATH = '/opt_path'
TOPIC_ODOM = '/aft_mapped_to_init'
TOPIC_IMU = '/livox/imu'
TOPIC_CMD_VEL = '/cmd_vel'

# Performance Tuning
UI_BUFFER_LEN = 300     # Keep ~10s data for Live UI
UPDATE_INTERVAL_MS = 50 # 20 FPS
RECORD_RATE_HZ = 30.0   # 30Hz Sampling

# Determine paths relative to this script
SCRIPT_PATH = os.path.abspath(__file__)
SCRIPT_DIR = os.path.dirname(SCRIPT_PATH)
PARENT_DIR = os.path.dirname(SCRIPT_DIR) # ../data_analyzer/
CSV_DIR = os.path.join(PARENT_DIR, 'csv')
IMG_DIR = os.path.join(PARENT_DIR, 'data')

# Theme
THEME = {
    'bg': '#1e1e2e', 'plot_bg': '#252535', 'text': '#cdd6f4', 'text_dim': '#6c7086',
    'grid': '#45475a', 'pos': '#f38ba8', 'vel': '#fab387', 'acc': '#f9e2af',
    'plan': '#89b4fa', 'freq': '#a6e3a1', 'ok': '#a6e3a1', 'err': '#f38ba8',
    'traj_robot': '#f5c2e7', 'traj_plan': '#89b4fa', 'fill_alpha': 0.2,
    'comp_x': '#89dceb', 'comp_y': '#cba6f7' # Cyan and Purple for X/Y components
}

def quaternion_to_yaw(q):
    """Calculate yaw from quaternion (w, x, y, z)"""
    # siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    # cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    # ROS2 msg usually has x,y,z,w
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)

class Nav2Analyzer(Node):
    def __init__(self):
        super().__init__('nav2_performance_analyzer')
        self.lock = threading.Lock()
        self.running = True
        
        # --- Live Data Buffers (Deque for UI) ---
        self.times = deque(maxlen=UI_BUFFER_LEN)
        
        # Position Errors
        self.pos_errors = deque(maxlen=UI_BUFFER_LEN)
        self.pos_errors_x = deque(maxlen=UI_BUFFER_LEN)
        self.pos_errors_y = deque(maxlen=UI_BUFFER_LEN)
        
        # Velocity Errors
        self.vel_errors = deque(maxlen=UI_BUFFER_LEN)
        self.vel_errors_x = deque(maxlen=UI_BUFFER_LEN) # New
        self.vel_errors_y = deque(maxlen=UI_BUFFER_LEN) # New
        
        # Acceleration Errors
        self.acc_errors = deque(maxlen=UI_BUFFER_LEN)
        self.acc_errors_x = deque(maxlen=UI_BUFFER_LEN) # New
        self.acc_errors_y = deque(maxlen=UI_BUFFER_LEN) # New
        
        self.plan_freqs = deque(maxlen=UI_BUFFER_LEN)
        self.ctrl_freqs = deque(maxlen=UI_BUFFER_LEN)
        
        # Trajectory Buffers
        self.robot_x = deque(maxlen=UI_BUFFER_LEN)
        self.robot_y = deque(maxlen=UI_BUFFER_LEN)
        self.latest_plan_path = ([], [])
        
        # --- State ---
        self.latest_plan_msg = None
        self.latest_odom_msg = None
        self.latest_imu_msg = None # Store full msg for vector calc
        
        # Freq Counters
        self.cmd_vel_count = 0
        self.plan_count = 0
        self.last_freq_time = time.time()
        self.curr_ctrl_freq = 0.0
        self.curr_plan_freq = 0.0
        
        self.start_time = time.time()
        self.topic_last_seen = {'Plan': 0, 'Odom': 0, 'IMU': 0, 'Cmd': 0}

        # --- CSV Logging Setup ---
        if not os.path.exists(CSV_DIR):
            os.makedirs(CSV_DIR)
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join(CSV_DIR, f"nav2_log_{timestamp}.csv")
        
        try:
            self.csv_file = open(self.csv_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'Time', 
                'Pos_Error', 'Pos_Err_X', 'Pos_Err_Y', 
                'Vel_Error', 'Vel_Err_X', 'Vel_Err_Y',
                'Acc_Error', 'Acc_Err_X', 'Acc_Err_Y',
                'Plan_Freq', 'Ctrl_Freq', 'Robot_X', 'Robot_Y'
            ])
            self.get_logger().info(f"Logging CSV to: {self.csv_filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to open CSV file: {e}")
            self.csv_file = None

        # --- Subscribers ---
        self.create_subscription(MpcPositionCommand, TOPIC_OPT_PATH, self.plan_cb, 1)
        self.create_subscription(Odometry, TOPIC_ODOM, self.odom_cb, 10)
        self.create_subscription(Imu, TOPIC_IMU, self.imu_cb, 10)
        self.create_subscription(Twist, TOPIC_CMD_VEL, self.cmd_vel_cb, 10)
        
        # Timer
        self.create_timer(1.0 / RECORD_RATE_HZ, self.loop)

    def plan_cb(self, msg):
        with self.lock:
            self.latest_plan_msg = msg
            self.plan_count += 1
            self.topic_last_seen['Plan'] = time.time()
            px = [p.position.x for p in msg.cmds]
            py = [p.position.y for p in msg.cmds]
            self.latest_plan_path = (px, py)

    def imu_cb(self, msg):
        with self.lock:
            self.latest_imu_msg = msg
            self.topic_last_seen['IMU'] = time.time()

    def cmd_vel_cb(self, msg):
        with self.lock:
            self.cmd_vel_count += 1
            self.topic_last_seen['Cmd'] = time.time()

    def odom_cb(self, msg):
        with self.lock:
            self.latest_odom_msg = msg
            self.topic_last_seen['Odom'] = time.time()

    def loop(self):
        """High-freq data processing loop"""
        if not self.running: return
        now = time.time()
        
        with self.lock:
            # 1. Frequency Calc
            if now - self.last_freq_time >= 1.0:
                dt = now - self.last_freq_time
                self.curr_ctrl_freq = self.cmd_vel_count / dt
                self.curr_plan_freq = self.plan_count / dt
                self.cmd_vel_count = 0
                self.plan_count = 0
                self.last_freq_time = now

            # 2. Metrics Calculation
            pos_err, pos_err_x, pos_err_y = 0.0, 0.0, 0.0
            vel_err, vel_err_x, vel_err_y = 0.0, 0.0, 0.0
            acc_err, acc_err_x, acc_err_y = 0.0, 0.0, 0.0
            rx, ry = 0.0, 0.0
            
            has_data = (self.latest_odom_msg and self.latest_plan_msg and len(self.latest_plan_msg.cmds) > 0)
            
            if self.latest_odom_msg:
                rx = self.latest_odom_msg.pose.pose.position.x
                ry = self.latest_odom_msg.pose.pose.position.y

            if has_data:
                # 2.1 Coordinate Transform: Body -> World
                # Calculate Yaw
                yaw = quaternion_to_yaw(self.latest_odom_msg.pose.pose.orientation)
                cv, sv = np.cos(yaw), np.sin(yaw)

                # Transform Body Velocity to World
                b_vx = self.latest_odom_msg.twist.twist.linear.x
                b_vy = self.latest_odom_msg.twist.twist.linear.y
                w_vx = b_vx * cv - b_vy * sv
                w_vy = b_vx * sv + b_vy * cv
                
                # Transform Body Accel (IMU) to World
                # Note: IMU accel includes gravity, assuming planar motion and mostly kinematic part on X/Y
                if self.latest_imu_msg:
                    b_ax = self.latest_imu_msg.linear_acceleration.x
                    b_ay = self.latest_imu_msg.linear_acceleration.y
                    w_ax = b_ax * cv - b_ay * sv
                    w_ay = b_ax * sv + b_ay * cv
                    imu_norm = np.hypot(b_ax, b_ay)
                else:
                    w_ax, w_ay, imu_norm = 0.0, 0.0, 0.0

                # 2.2 Find Closest Point in Plan
                min_dist = float('inf')
                best_cmd = self.latest_plan_msg.cmds[0]
                search_horizon = min(len(self.latest_plan_msg.cmds), 50) 
                
                for i in range(search_horizon):
                    cmd = self.latest_plan_msg.cmds[i]
                    dist = np.hypot(cmd.position.x - rx, cmd.position.y - ry)
                    if dist < min_dist:
                        min_dist = dist
                        best_cmd = cmd
                
                # 2.3 Calculate Errors
                # Pos
                pos_err = min_dist
                pos_err_x = abs(best_cmd.position.x - rx)
                pos_err_y = abs(best_cmd.position.y - ry)
                
                # Vel
                ref_vel_norm = np.hypot(best_cmd.velocity.x, best_cmd.velocity.y)
                act_vel_norm = np.hypot(w_vx, w_vy)
                vel_err = abs(ref_vel_norm - act_vel_norm)
                vel_err_x = abs(best_cmd.velocity.x - w_vx)
                vel_err_y = abs(best_cmd.velocity.y - w_vy)
                
                # Acc
                ref_acc_norm = np.hypot(best_cmd.acceleration.x, best_cmd.acceleration.y)
                acc_err = abs(ref_acc_norm - imu_norm) # Total magnitude error
                acc_err_x = abs(best_cmd.acceleration.x - w_ax)
                acc_err_y = abs(best_cmd.acceleration.y - w_ay)

            # 3. Save to CSV
            t_rel = now - self.start_time
            if self.csv_file:
                self.csv_writer.writerow([
                    f"{t_rel:.3f}", 
                    f"{pos_err:.4f}", f"{pos_err_x:.4f}", f"{pos_err_y:.4f}",
                    f"{vel_err:.4f}", f"{vel_err_x:.4f}", f"{vel_err_y:.4f}",
                    f"{acc_err:.4f}", f"{acc_err_x:.4f}", f"{acc_err_y:.4f}",
                    f"{self.curr_plan_freq:.1f}", f"{self.curr_ctrl_freq:.1f}", 
                    f"{rx:.3f}", f"{ry:.3f}"
                ])

            # 4. Update UI Buffers
            self.times.append(t_rel)
            self.pos_errors.append(pos_err)
            self.pos_errors_x.append(pos_err_x)
            self.pos_errors_y.append(pos_err_y)
            
            self.vel_errors.append(vel_err)
            self.vel_errors_x.append(vel_err_x)
            self.vel_errors_y.append(vel_err_y)
            
            self.acc_errors.append(acc_err)
            self.acc_errors_x.append(acc_err_x)
            self.acc_errors_y.append(acc_err_y)
            
            self.plan_freqs.append(self.curr_plan_freq)
            self.ctrl_freqs.append(self.curr_ctrl_freq)
            self.robot_x.append(rx)
            self.robot_y.append(ry)

    def close(self):
        self.running = False
        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
            print(f"CSV Saved to: {self.csv_filename}")

def generate_post_run_report(csv_path):
    """Generates static plots from the saved CSV after exit."""
    print("Generating post-run analysis report...")
    if not os.path.exists(IMG_DIR):
        os.makedirs(IMG_DIR)

    try:
        import matplotlib
        matplotlib.use('Agg') 
        import matplotlib.pyplot as plt

        # Read CSV
        # Indices: 0:Time, 1:Pos, 2:Px, 3:Py, 4:Vel, 5:Vx, 6:Vy, 7:Acc, 8:Ax, 9:Ay, 10:PF, 11:CF, 12:Rx, 13:Ry
        data = {k: [] for k in ['t', 'p', 'px', 'py', 'v', 'vx', 'vy', 'a', 'ax', 'ay', 'rx', 'ry']}
        
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            next(reader) # Skip header
            for row in reader:
                try:
                    data['t'].append(float(row[0]))
                    data['p'].append(float(row[1]))
                    data['px'].append(float(row[2]))
                    data['py'].append(float(row[3]))
                    data['v'].append(float(row[4]))
                    data['vx'].append(float(row[5]))
                    data['vy'].append(float(row[6]))
                    data['a'].append(float(row[7]))
                    data['ax'].append(float(row[8]))
                    data['ay'].append(float(row[9]))
                    data['rx'].append(float(row[12]))
                    data['ry'].append(float(row[13]))
                except ValueError: continue
        
        if not data['t']:
            print("No data found in CSV to generate report.")
            return

        # Setup Plot
        fig = plt.figure(figsize=(16, 12), facecolor='white')
        gs = gridspec.GridSpec(3, 2)

        # 1. Trajectory
        ax_traj = fig.add_subplot(gs[:, 0])
        ax_traj.plot(data['rx'], data['ry'], label='Robot Path', color='blue', linewidth=1.5)
        ax_traj.set_title("Full Trajectory Map")
        ax_traj.set_xlabel("X (m)")
        ax_traj.set_ylabel("Y (m)")
        ax_traj.axis('equal')
        ax_traj.grid(True, linestyle=':', alpha=0.6)
        ax_traj.legend()

        # 2. Pos Error
        ax_pos = fig.add_subplot(gs[0, 1])
        ax_pos.plot(data['t'], data['p'], color='red', linewidth=1.5, label='Total')
        ax_pos.plot(data['t'], data['px'], color='cyan', linewidth=0.8, alpha=0.7, label='X')
        ax_pos.plot(data['t'], data['py'], color='magenta', linewidth=0.8, alpha=0.7, label='Y')
        ax_pos.set_title(f"Position Error (Mean: {np.mean(data['p']):.4f}m)")
        ax_pos.set_ylabel("Error (m)")
        ax_pos.legend(fontsize='small')
        ax_pos.grid(True, linestyle=':', alpha=0.6)

        # 3. Vel Error
        ax_vel = fig.add_subplot(gs[1, 1], sharex=ax_pos)
        ax_vel.plot(data['t'], data['v'], color='orange', linewidth=1.5, label='Total')
        ax_vel.plot(data['t'], data['vx'], color='cyan', linewidth=0.8, alpha=0.7, label='X')
        ax_vel.plot(data['t'], data['vy'], color='magenta', linewidth=0.8, alpha=0.7, label='Y')
        ax_vel.set_title("Velocity Error")
        ax_vel.set_ylabel("Error (m/s)")
        ax_vel.legend(fontsize='small')
        ax_vel.grid(True, linestyle=':', alpha=0.6)

        # 4. Acc Error
        ax_acc = fig.add_subplot(gs[2, 1], sharex=ax_pos)
        ax_acc.plot(data['t'], data['a'], color='green', linewidth=1.5, label='Total')
        ax_acc.plot(data['t'], data['ax'], color='cyan', linewidth=0.8, alpha=0.7, label='X')
        ax_acc.plot(data['t'], data['ay'], color='magenta', linewidth=0.8, alpha=0.7, label='Y')
        ax_acc.set_title("Acceleration Error")
        ax_acc.set_ylabel("Error (m/s²)")
        ax_acc.set_xlabel("Time (s)")
        ax_acc.legend(fontsize='small')
        ax_acc.grid(True, linestyle=':', alpha=0.6)

        # Save
        timestamp = os.path.basename(csv_path).replace('nav2_log_', '').replace('.csv', '')
        out_path = os.path.join(IMG_DIR, f"report_{timestamp}.png")
        
        plt.tight_layout()
        plt.savefig(out_path, dpi=150)
        plt.close(fig)
        print(f"Report image saved to: {out_path}")

    except Exception as e:
        print(f"Failed to generate report: {e}")

class ModernVisualizer:
    def __init__(self, node):
        self.node = node
        self.paused = False

        plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial']
        plt.rcParams['axes.unicode_minus'] = False
        plt.rcParams['toolbar'] = 'None'

        self.fig = plt.figure(figsize=(14, 9), facecolor=THEME['bg'])
        self.fig.canvas.manager.set_window_title('NAV2 Minco Analyzer (Real-time)')
        
        # Grid layout
        gs = gridspec.GridSpec(4, 2, width_ratios=[1.2, 1], hspace=0.4, wspace=0.15,
                               top=0.92, bottom=0.08, left=0.05, right=0.95)

        # Header
        self.ax_header = self.fig.add_axes([0.0, 0.92, 1.0, 0.08], facecolor=THEME['bg'])
        self.ax_header.axis('off')
        self._init_header()

        # Left: Trajectory
        self.ax_traj = self.fig.add_subplot(gs[0:3, 0], facecolor=THEME['plot_bg'])
        self._style_axis(self.ax_traj, "Real-time Trajectory", "Y (m)", "X (m)")
        self.ax_traj.axis('equal')
        self.ln_plan, = self.ax_traj.plot([], [], color=THEME['plan'], lw=1.5, ls='--')
        self.ln_robot, = self.ax_traj.plot([], [], color=THEME['traj_robot'], lw=2)

        # Left Bottom: Frequency
        self.ax_freq = self.fig.add_subplot(gs[3, 0], facecolor=THEME['plot_bg'])
        self._style_axis(self.ax_freq, "System Frequency", "Hz")
        self.ln_freq_c, = self.ax_freq.plot([], [], color=THEME['freq'], lw=1.5, label='Ctrl')
        self.ln_freq_p, = self.ax_freq.plot([], [], color=THEME['plan'], lw=1.5, ls='--', label='Plan')
        self.ax_freq.legend(loc='upper left', frameon=False, labelcolor=THEME['text_dim'], fontsize=8, ncol=2)

        # Right: Errors
        # 1. Pos Error
        self.ax_pos = self.fig.add_subplot(gs[0:2, 1], facecolor=THEME['plot_bg'])
        self._style_axis(self.ax_pos, "Position Error (Tot/X/Y)", "m")
        self.ln_pos_x, = self.ax_pos.plot([], [], color=THEME['comp_x'], lw=1, alpha=0.8, label='X')
        self.ln_pos_y, = self.ax_pos.plot([], [], color=THEME['comp_y'], lw=1, alpha=0.8, label='Y')
        self.ln_pos, = self.ax_pos.plot([], [], color=THEME['pos'], lw=2, label='Total')
        self.ax_pos.legend(loc='upper right', frameon=False, labelcolor=THEME['text_dim'], fontsize=8, ncol=3)

        # 2. Vel Error
        self.ax_vel = self.fig.add_subplot(gs[2, 1], facecolor=THEME['plot_bg'], sharex=self.ax_pos)
        self._style_axis(self.ax_vel, "Velocity Error", "m/s")
        self.ln_vel_x, = self.ax_vel.plot([], [], color=THEME['comp_x'], lw=1, alpha=0.8)
        self.ln_vel_y, = self.ax_vel.plot([], [], color=THEME['comp_y'], lw=1, alpha=0.8)
        self.ln_vel, = self.ax_vel.plot([], [], color=THEME['vel'], lw=1.5)

        # 3. Acc Error
        self.ax_acc = self.fig.add_subplot(gs[3, 1], facecolor=THEME['plot_bg'], sharex=self.ax_pos)
        self._style_axis(self.ax_acc, "Acceleration Error", "m/s²")
        self.ln_acc_x, = self.ax_acc.plot([], [], color=THEME['comp_x'], lw=1, alpha=0.8)
        self.ln_acc_y, = self.ax_acc.plot([], [], color=THEME['comp_y'], lw=1, alpha=0.8)
        self.ln_acc, = self.ax_acc.plot([], [], color=THEME['acc'], lw=1.5)

        # Store fills to remove later
        self.fills = {'pos': None, 'vel': None, 'acc': None}

        # Buttons
        ax_b1 = plt.axes([0.9, 0.94, 0.08, 0.04])
        self.btn_pause = Button(ax_b1, 'Pause', color=THEME['bg'], hovercolor=THEME['grid'])
        self.btn_pause.label.set_color(THEME['text'])
        self.btn_pause.on_clicked(self.toggle_pause)

        self.ani = FuncAnimation(self.fig, self.update, interval=UPDATE_INTERVAL_MS, blit=False)
        plt.show()

    def _init_header(self):
        self.ax_header.text(0.02, 0.5, "NAV2 ANALYZER", color=THEME['text'], fontsize=14, fontweight='bold', va='center')
        self.txt_status = self.ax_header.text(0.18, 0.5, "● INIT", color=THEME['text_dim'], fontsize=10, va='center')
        self.stat_pos = self._add_stat(0.40, "POS ERR Tot/X/Y (m)", THEME['pos'])
        self.stat_vel = self._add_stat(0.55, "VEL ERR Tot/X/Y (m/s)", THEME['vel'])
        self.stat_acc = self._add_stat(0.70, "ACC ERR Tot/X/Y (m/s²)", THEME['acc'])
        self.stat_freq = self._add_stat(0.85, "FREQ (Hz)", THEME['freq'])

    def _add_stat(self, x, label, color):
        self.ax_header.text(x, 0.7, label, color=THEME['text_dim'], fontsize=7, ha='left')
        return self.ax_header.text(x, 0.25, "0.00", color=color, fontsize=11, fontweight='bold', ha='left', family='monospace')

    def _style_axis(self, ax, title, ylabel, xlabel=None):
        ax.set_title(title, color=THEME['text'], loc='left', fontsize=9, pad=3)
        ax.set_ylabel(ylabel, color=THEME['text_dim'], fontsize=8)
        if xlabel: ax.set_xlabel(xlabel, color=THEME['text_dim'], fontsize=8)
        ax.tick_params(axis='both', colors=THEME['text_dim'], labelsize=7)
        for s in ax.spines.values(): s.set_visible(False)
        ax.spines['bottom'].set_visible(True)
        ax.spines['bottom'].set_color(THEME['grid'])
        ax.spines['left'].set_visible(True)
        ax.spines['left'].set_color(THEME['grid'])
        ax.grid(True, color=THEME['grid'], ls=':', lw=0.5, alpha=0.5)

    def toggle_pause(self, event):
        self.paused = not self.paused

    def update(self, frame):
        if self.paused: return

        with self.node.lock:
            if len(self.node.times) < 2: return
            
            times = np.array(self.node.times)
            # Pos
            pos = np.array(self.node.pos_errors)
            pos_x = np.array(self.node.pos_errors_x)
            pos_y = np.array(self.node.pos_errors_y)
            # Vel
            vel = np.array(self.node.vel_errors)
            vel_x = np.array(self.node.vel_errors_x)
            vel_y = np.array(self.node.vel_errors_y)
            # Acc
            acc = np.array(self.node.acc_errors)
            acc_x = np.array(self.node.acc_errors_x)
            acc_y = np.array(self.node.acc_errors_y)
            
            pf = np.array(self.node.plan_freqs)
            cf = np.array(self.node.ctrl_freqs)
            rx = list(self.node.robot_x)
            ry = list(self.node.robot_y)
            px, py = self.node.latest_plan_path

        # Update Stats (Total + X/Y)
        self.stat_pos.set_text(f"{pos[-1]:.3f} ({pos_x[-1]:.2f}/{pos_y[-1]:.2f})")
        self.stat_vel.set_text(f"{vel[-1]:.2f} ({vel_x[-1]:.2f}/{vel_y[-1]:.2f})")
        self.stat_acc.set_text(f"{acc[-1]:.2f} ({acc_x[-1]:.2f}/{acc_y[-1]:.2f})")
        self.stat_freq.set_text(f"{cf[-1]:.0f}")
        
        now = time.time()
        is_conn = (now - self.node.topic_last_seen['Odom']) < 2.0
        self.txt_status.set_text("● ONLINE" if is_conn else "○ WAITING")
        self.txt_status.set_color(THEME['ok'] if is_conn else THEME['err'])

        # Update Plots
        if len(rx) > 0:
            self.ln_robot.set_data(rx, ry)
            cx, cy = rx[-1], ry[-1]
            self.ax_traj.set_xlim(cx - 5, cx + 5)
            self.ax_traj.set_ylim(cy - 5, cy + 5)
        
        self.ln_plan.set_data(px, py)
        
        # Pos Lines
        self.ln_pos.set_data(times, pos)
        self.ln_pos_x.set_data(times, pos_x)
        self.ln_pos_y.set_data(times, pos_y)
        
        # Vel Lines
        self.ln_vel.set_data(times, vel)
        self.ln_vel_x.set_data(times, vel_x)
        self.ln_vel_y.set_data(times, vel_y)
        
        # Acc Lines
        self.ln_acc.set_data(times, acc)
        self.ln_acc_x.set_data(times, acc_x)
        self.ln_acc_y.set_data(times, acc_y)
        
        self.ln_freq_c.set_data(times, cf)
        self.ln_freq_p.set_data(times, pf)

        t_min, t_max = times[0], times[-1]
        for ax in [self.ax_pos, self.ax_vel, self.ax_acc, self.ax_freq]:
            ax.set_xlim(t_min, t_max)

        self.ax_pos.set_ylim(0, max(0.1, pos.max() * 1.2))
        self.ax_vel.set_ylim(0, max(0.1, vel.max() * 1.2))
        self.ax_acc.set_ylim(0, max(0.1, acc.max() * 1.2))
        self.ax_freq.set_ylim(0, max(30, max(cf.max(), pf.max()) * 1.2))

        try:
            for key, ax, data, color in [
                ('pos', self.ax_pos, pos, THEME['pos']),
                ('vel', self.ax_vel, vel, THEME['vel']),
                ('acc', self.ax_acc, acc, THEME['acc'])
            ]:
                if self.fills[key]: self.fills[key].remove()
                self.fills[key] = ax.fill_between(times, 0, data, color=color, alpha=0.2)
        except: pass

def main():
    rclpy.init()
    
    # Setup Node
    node = Nav2Analyzer()
    
    # Start ROS thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    # Run UI
    try:
        viz = ModernVisualizer(node)
    except KeyboardInterrupt:
        print("\nCaught KeyboardInterrupt in UI.")
    except Exception as e:
        print(f"\nError in UI: {e}")
    finally:
        # CLEAN SHUTDOWN SEQUENCE
        print("\nShutting down...")
        node.close()  # Save CSV
        
        # Stop matplotlib cleanly
        plt.close('all') 
        
        if rclpy.ok():
            rclpy.shutdown()
        
        # Generate Report AFTER shutdown to avoid thread conflicts
        generate_post_run_report(node.csv_filename)

if __name__ == '__main__':
    main()