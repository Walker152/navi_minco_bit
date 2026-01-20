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

# --- Sensor Installation Parameters (User Defined) ---
# Radar/IMU installed at: Right 20cm (y = -0.2), Roll = 20 degrees
SENSOR_OFFSET_Y = -0.20  # meters (Right is negative Y in ROS body frame usually, strictly: Y is Left)
SENSOR_ROLL_DEG = 20.0   # degrees

# Performance Tuning
RECORD_RATE_HZ = 30.0   
UI_BUFFER_LEN = int(30 * RECORD_RATE_HZ) # Keep 30 seconds of data for Live UI
UPDATE_INTERVAL_MS = 33 

# Determine paths
SCRIPT_PATH = os.path.abspath(__file__)
SCRIPT_DIR = os.path.dirname(SCRIPT_PATH)
PARENT_DIR = os.path.dirname(SCRIPT_DIR)
CSV_DIR = os.path.join(PARENT_DIR, 'csv')
IMG_DIR = os.path.join(PARENT_DIR, 'data')

# Theme
THEME = {
    'bg': '#1e1e2e', 'plot_bg': '#252535', 'text': '#cdd6f4', 'text_dim': '#6c7086',
    'grid': '#45475a', 'pos': '#f38ba8', 'vel': '#fab387', 'acc': '#f9e2af',
    'plan': '#89b4fa', 'real': '#f38ba8', 'freq': '#a6e3a1', 'ok': '#a6e3a1', 'err': '#f38ba8',
    'traj_robot': '#f5c2e7', 'traj_plan': '#89b4fa', 'fill_alpha': 0.1,
    'x_col': '#89dceb', 'y_col': '#cba6f7' 
}

def quaternion_to_yaw(q):
    """Calculate yaw from quaternion (w, x, y, z)"""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)

def get_rotation_matrix_x(deg):
    """Rotation matrix around X axis"""
    rad = np.radians(deg)
    c, s = np.cos(rad), np.sin(rad)
    return np.array([
        [1, 0,  0],
        [0, c, -s],
        [0, s,  c]
    ])

class Nav2Analyzer(Node):
    def __init__(self):
        super().__init__('nav2_performance_analyzer')
        self.lock = threading.Lock()
        self.running = True
        
        # --- IMU Integration State ---
        self.imu_vel_body = np.array([0.0, 0.0]) # [vx, vy] in Body Frame
        self.last_imu_time = None
        # Precompute rotation matrix for sensor mounting (Roll 20 deg)
        self.R_sensor_to_base = get_rotation_matrix_x(SENSOR_ROLL_DEG)
        
        # --- Live Data Buffers ---
        self.times = deque(maxlen=UI_BUFFER_LEN)
        
        # Velocity Comparison
        self.plan_vel_x = deque(maxlen=UI_BUFFER_LEN)
        self.real_vel_x = deque(maxlen=UI_BUFFER_LEN)
        self.plan_vel_y = deque(maxlen=UI_BUFFER_LEN)
        self.real_vel_y = deque(maxlen=UI_BUFFER_LEN)
        
        # Acceleration Comparison
        self.plan_acc_x = deque(maxlen=UI_BUFFER_LEN)
        self.real_acc_x = deque(maxlen=UI_BUFFER_LEN)
        self.plan_acc_y = deque(maxlen=UI_BUFFER_LEN)
        self.real_acc_y = deque(maxlen=UI_BUFFER_LEN)
        
        # Frequency
        self.plan_freqs = deque(maxlen=UI_BUFFER_LEN)
        self.ctrl_freqs = deque(maxlen=UI_BUFFER_LEN)
        
        # Trajectory Buffers (30s Window)
        self.robot_x = deque(maxlen=UI_BUFFER_LEN)
        self.robot_y = deque(maxlen=UI_BUFFER_LEN)
        self.latest_plan_path = ([], [])
        
        # --- State ---
        self.latest_plan_msg = None
        self.latest_odom_msg = None
        
        # Store current REAL kinematic state (calculated from IMU)
        self.curr_real_vel_world = np.array([0.0, 0.0])
        self.curr_real_acc_world = np.array([0.0, 0.0])
        
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
                'Plan_Vel_X', 'Real_Vel_X', 'Plan_Vel_Y', 'Real_Vel_Y',
                'Plan_Acc_X', 'Real_Acc_X', 'Plan_Acc_Y', 'Real_Acc_Y',
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
        """
        Process IMU data with Roll correction and Integration
        """
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        a_sensor = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        w_z = msg.angular_velocity.z 

        with self.lock:
            # Rotate to Body Frame
            a_body_raw = self.R_sensor_to_base @ a_sensor
            # Remove Gravity
            a_body_lin = a_body_raw - np.array([0.0, 0.0, 9.80665])
            
            # Integrate
            if self.last_imu_time is not None:
                dt = now - self.last_imu_time
                if dt > 0 and dt < 0.2:
                    self.imu_vel_body[0] += a_body_lin[0] * dt
                    self.imu_vel_body[1] += a_body_lin[1] * dt
            
            self.last_imu_time = now
            self.topic_last_seen['IMU'] = time.time()

            # Lever Arm Correction
            v_center_x = self.imu_vel_body[0] + w_z * SENSOR_OFFSET_Y
            v_center_y = self.imu_vel_body[1]
            
            # Transform to World
            if self.latest_odom_msg:
                yaw = quaternion_to_yaw(self.latest_odom_msg.pose.pose.orientation)
                c, s = np.cos(yaw), np.sin(yaw)
                
                self.curr_real_vel_world[0] = v_center_x * c - v_center_y * s
                self.curr_real_vel_world[1] = v_center_x * s + v_center_y * c
                
                ac_x = a_body_lin[0]
                ac_y = a_body_lin[1] - (w_z**2 * SENSOR_OFFSET_Y)
                
                self.curr_real_acc_world[0] = ac_x * c - ac_y * s
                self.curr_real_acc_world[1] = ac_x * s + ac_y * c

    def cmd_vel_cb(self, msg):
        with self.lock:
            self.cmd_vel_count += 1
            self.topic_last_seen['Cmd'] = time.time()

    def odom_cb(self, msg):
        with self.lock:
            self.latest_odom_msg = msg
            self.topic_last_seen['Odom'] = time.time()
            
            # ZUPT
            odom_v = np.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
            if odom_v < 0.01:
                self.imu_vel_body[:] = 0.0

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

            # 2. Compare with Plan
            pv_x, rv_x, pv_y, rv_y = 0.0, 0.0, 0.0, 0.0
            pa_x, ra_x, pa_y, ra_y = 0.0, 0.0, 0.0, 0.0
            rx, ry = 0.0, 0.0
            
            has_data = (self.latest_odom_msg and self.latest_plan_msg and len(self.latest_plan_msg.cmds) > 0)
            
            if self.latest_odom_msg:
                rx = self.latest_odom_msg.pose.pose.position.x
                ry = self.latest_odom_msg.pose.pose.position.y

            if has_data:
                min_dist = float('inf')
                best_cmd = self.latest_plan_msg.cmds[0]
                search_horizon = min(len(self.latest_plan_msg.cmds), 50) 
                
                for i in range(search_horizon):
                    cmd = self.latest_plan_msg.cmds[i]
                    dist = np.hypot(cmd.position.x - rx, cmd.position.y - ry)
                    if dist < min_dist:
                        min_dist = dist
                        best_cmd = cmd
                
                pv_x, pv_y = best_cmd.velocity.x, best_cmd.velocity.y
                pa_x, pa_y = best_cmd.acceleration.x, best_cmd.acceleration.y
                
                rv_x, rv_y = self.curr_real_vel_world[0], self.curr_real_vel_world[1]
                ra_x, ra_y = self.curr_real_acc_world[0], self.curr_real_acc_world[1]

            # 3. Save to CSV
            if self.latest_odom_msg:
                t_rel = self.latest_odom_msg.header.stamp.sec + self.latest_odom_msg.header.stamp.nanosec*1e-9 - self.start_time
            else:
                t_rel = now - self.start_time

            if self.csv_file:
                self.csv_writer.writerow([
                    f"{t_rel:.3f}", 
                    f"{pv_x:.4f}", f"{rv_x:.4f}", f"{pv_y:.4f}", f"{rv_y:.4f}",
                    f"{pa_x:.4f}", f"{ra_x:.4f}", f"{pa_y:.4f}", f"{ra_y:.4f}",
                    f"{self.curr_plan_freq:.1f}", f"{self.curr_ctrl_freq:.1f}", 
                    f"{rx:.3f}", f"{ry:.3f}"
                ])

            # 4. Update UI Buffers
            self.times.append(t_rel)
            
            self.plan_vel_x.append(pv_x)
            self.real_vel_x.append(rv_x)
            self.plan_vel_y.append(pv_y)
            self.real_vel_y.append(rv_y)
            
            self.plan_acc_x.append(pa_x)
            self.real_acc_x.append(ra_x)
            self.plan_acc_y.append(pa_y)
            self.real_acc_y.append(ra_y)
            
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
    print("Generating post-run analysis report...")
    if not os.path.exists(IMG_DIR): os.makedirs(IMG_DIR)

    try:
        import matplotlib
        matplotlib.use('Agg') 
        import matplotlib.pyplot as plt

        data = {k: [] for k in ['t', 'pvx', 'rvx', 'pvy', 'rvy', 'pax', 'rax', 'pay', 'ray', 'rx', 'ry']}
        
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            next(reader) 
            for row in reader:
                try:
                    data['t'].append(float(row[0]))
                    data['pvx'].append(float(row[1]))
                    data['rvx'].append(float(row[2]))
                    data['pvy'].append(float(row[3]))
                    data['rvy'].append(float(row[4]))
                    data['pax'].append(float(row[5]))
                    data['rax'].append(float(row[6]))
                    data['pay'].append(float(row[7]))
                    data['ray'].append(float(row[8]))
                    data['rx'].append(float(row[11]))
                    data['ry'].append(float(row[12]))
                except ValueError: continue
        
        if not data['t']: return

        fig = plt.figure(figsize=(16, 12), facecolor='white')
        gs = gridspec.GridSpec(3, 2)

        # Swapped Axis for Report (Vertical=X, Horizontal=Y)
        ax_traj = fig.add_subplot(gs[:, 0])
        ax_traj.plot(data['ry'], data['rx'], label='Robot Path', color='blue', linewidth=1.5)
        ax_traj.set_title("Full Trajectory (X=Vert, Y=Horiz)")
        ax_traj.set_xlabel("Y (m)")
        ax_traj.set_ylabel("X (m)")
        ax_traj.axis('equal')
        ax_traj.grid(True, alpha=0.5)
        ax_traj.legend()

        ax_vx = fig.add_subplot(gs[0, 1])
        ax_vx.plot(data['t'], data['pvx'], 'b--', alpha=0.7, label='Plan')
        ax_vx.plot(data['t'], data['rvx'], 'b', alpha=1.0, label='Real (IMU)')
        ax_vx.set_title("Vel X")
        ax_vx.grid(True, alpha=0.5)
        ax_vx.legend()

        ax_vy = fig.add_subplot(gs[1, 1], sharex=ax_vx)
        ax_vy.plot(data['t'], data['pvy'], color='purple', ls='--', alpha=0.7, label='Plan')
        ax_vy.plot(data['t'], data['rvy'], color='purple', alpha=1.0, label='Real (IMU)')
        ax_vy.set_title("Vel Y")
        ax_vy.grid(True, alpha=0.5)

        ax_ax = fig.add_subplot(gs[2, 1], sharex=ax_vx)
        ax_ax.plot(data['t'], data['pax'], color='green', ls='--', alpha=0.7, label='Plan X')
        ax_ax.plot(data['t'], data['rax'], color='green', alpha=1.0, label='Real X (IMU)')
        ax_ax.set_title("Acc X")
        ax_ax.grid(True, alpha=0.5)

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
        self.fig.canvas.manager.set_window_title('NAV2 Minco Analyzer (IMU Integration Mode)')
        self.fig.canvas.mpl_connect('close_event', self.on_close)
        
        gs = gridspec.GridSpec(4, 2, width_ratios=[1.2, 1], hspace=0.4, wspace=0.15,
                               top=0.92, bottom=0.08, left=0.05, right=0.95)

        self.ax_header = self.fig.add_axes([0.0, 0.92, 1.0, 0.08], facecolor=THEME['bg'])
        self.ax_header.axis('off')
        self._init_header()

        # Left: Trajectory (SWAPPED AXES)
        # Vertical axis = X, Horizontal axis = Y
        self.ax_traj = self.fig.add_subplot(gs[0:3, 0], facecolor=THEME['plot_bg'])
        self._style_axis(self.ax_traj, "Trajectory (X=Vert, Y=Horiz)", "Y (m)", "X (m)")
        self.ax_traj.axis('equal')
        # Note: plot(y, x) instead of plot(x, y)
        self.ln_plan, = self.ax_traj.plot([], [], color=THEME['plan'], lw=1.5, ls='--')
        self.ln_robot, = self.ax_traj.plot([], [], color=THEME['traj_robot'], lw=2)

        self.ax_freq = self.fig.add_subplot(gs[3, 0], facecolor=THEME['plot_bg'])
        self._style_axis(self.ax_freq, "Frequency", "Hz")
        self.ln_freq_c, = self.ax_freq.plot([], [], color=THEME['freq'], lw=1.5, label='Ctrl')
        self.ln_freq_p, = self.ax_freq.plot([], [], color=THEME['plan'], lw=1.5, ls='--', label='Plan')
        self.ax_freq.legend(loc='upper left', frameon=False, labelcolor=THEME['text_dim'], fontsize=8)

        # Velocity Plots
        self.ax_vel_x = self.fig.add_subplot(gs[0, 1], facecolor=THEME['plot_bg'])
        self._style_axis(self.ax_vel_x, "Velocity X (IMU Int.)", "m/s")
        self.ln_pv_x, = self.ax_vel_x.plot([], [], color=THEME['x_col'], ls='--', alpha=0.8, label='Plan')
        self.ln_rv_x, = self.ax_vel_x.plot([], [], color=THEME['x_col'], lw=2, label='Real')
        self.ax_vel_x.legend(loc='upper right', frameon=False, labelcolor=THEME['text_dim'], fontsize=8)

        self.ax_vel_y = self.fig.add_subplot(gs[1, 1], facecolor=THEME['plot_bg'], sharex=self.ax_vel_x)
        self._style_axis(self.ax_vel_y, "Velocity Y (IMU Int.)", "m/s")
        self.ln_pv_y, = self.ax_vel_y.plot([], [], color=THEME['y_col'], ls='--', alpha=0.8)
        self.ln_rv_y, = self.ax_vel_y.plot([], [], color=THEME['y_col'], lw=2)

        # Accel Plots
        self.ax_acc_x = self.fig.add_subplot(gs[2, 1], facecolor=THEME['plot_bg'], sharex=self.ax_vel_x)
        self._style_axis(self.ax_acc_x, "Accel X (IMU)", "m/s²")
        self.ln_pa_x, = self.ax_acc_x.plot([], [], color=THEME['x_col'], ls='--', alpha=0.8)
        self.ln_ra_x, = self.ax_acc_x.plot([], [], color=THEME['x_col'], lw=2)

        self.ax_acc_y = self.fig.add_subplot(gs[3, 1], facecolor=THEME['plot_bg'], sharex=self.ax_vel_x)
        self._style_axis(self.ax_acc_y, "Accel Y (IMU)", "m/s²")
        self.ln_pa_y, = self.ax_acc_y.plot([], [], color=THEME['y_col'], ls='--', alpha=0.8)
        self.ln_ra_y, = self.ax_acc_y.plot([], [], color=THEME['y_col'], lw=2)

        ax_b1 = plt.axes([0.9, 0.94, 0.08, 0.04])
        self.btn_pause = Button(ax_b1, 'Pause', color=THEME['bg'], hovercolor=THEME['grid'])
        self.btn_pause.label.set_color(THEME['text'])
        self.btn_pause.on_clicked(self.toggle_pause)

        self.ani = FuncAnimation(self.fig, self.update, interval=UPDATE_INTERVAL_MS, blit=False)
        plt.show()

    def _init_header(self):
        self.ax_header.text(0.02, 0.5, "NAV2 ANALYZER", color=THEME['text'], fontsize=14, fontweight='bold', va='center')
        self.txt_status = self.ax_header.text(0.18, 0.5, "● INIT", color=THEME['text_dim'], fontsize=10, va='center')
        self.stat_vx = self._add_stat(0.35, "VEL X (m/s)", THEME['x_col'])
        self.stat_vy = self._add_stat(0.50, "VEL Y (m/s)", THEME['y_col'])
        self.stat_ax = self._add_stat(0.65, "ACC X (m/s²)", THEME['x_col'])
        self.stat_ay = self._add_stat(0.80, "ACC Y (m/s²)", THEME['y_col'])

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

    def on_close(self, event):
        self.node.close()
        plt.close(self.fig)

    def update(self, frame):
        if self.paused: return

        with self.node.lock:
            if len(self.node.times) < 2: return
            times = np.array(self.node.times)
            
            pvx, rvx = np.array(self.node.plan_vel_x), np.array(self.node.real_vel_x)
            pvy, rvy = np.array(self.node.plan_vel_y), np.array(self.node.real_vel_y)
            pax, rax = np.array(self.node.plan_acc_x), np.array(self.node.real_acc_x)
            pay, ray = np.array(self.node.plan_acc_y), np.array(self.node.real_acc_y)
            
            pf = np.array(self.node.plan_freqs)
            cf = np.array(self.node.ctrl_freqs)
            rx, ry = list(self.node.robot_x), list(self.node.robot_y)
            px, py = self.node.latest_plan_path

        self.stat_vx.set_text(f"{rvx[-1]:.2f}/{pvx[-1]:.2f}")
        self.stat_vy.set_text(f"{rvy[-1]:.2f}/{pvy[-1]:.2f}")
        self.stat_ax.set_text(f"{rax[-1]:.2f}/{pax[-1]:.2f}")
        self.stat_ay.set_text(f"{ray[-1]:.2f}/{pay[-1]:.2f}")
        
        now = time.time()
        is_conn = (now - self.node.topic_last_seen['Odom']) < 2.0
        self.txt_status.set_text("● ONLINE" if is_conn else "○ WAITING")
        self.txt_status.set_color(THEME['ok'] if is_conn else THEME['err'])

        # Fix Trajectory Viz: Swapped X/Y
        # Plot Y on Horizontal, X on Vertical
        if len(rx) > 0:
            self.ln_robot.set_data(ry, rx) 
            cx, cy = rx[-1], ry[-1]
            self.ax_traj.set_xlim(cy - 5, cy + 5) # Horizontal is Y
            self.ax_traj.set_ylim(cx - 5, cx + 5) # Vertical is X
        
        # Plan path also swapped
        self.ln_plan.set_data(py, px)
        
        self.ln_pv_x.set_data(times, pvx)
        self.ln_rv_x.set_data(times, rvx)
        self.ln_pv_y.set_data(times, pvy)
        self.ln_rv_y.set_data(times, rvy)
        
        self.ln_pa_x.set_data(times, pax)
        self.ln_ra_x.set_data(times, rax)
        self.ln_pa_y.set_data(times, pay)
        self.ln_ra_y.set_data(times, ray)
        
        self.ln_freq_c.set_data(times, cf)
        self.ln_freq_p.set_data(times, pf)

        t_min, t_max = times[0], times[-1]
        for ax in [self.ax_vel_x, self.ax_vel_y, self.ax_acc_x, self.ax_acc_y, self.ax_freq]:
            ax.set_xlim(t_min, t_max)

        for ax, d1, d2 in [
            (self.ax_vel_x, pvx, rvx), (self.ax_vel_y, pvy, rvy),
            (self.ax_acc_x, pax, rax), (self.ax_acc_y, pay, ray)
        ]:
            if len(d1) > 0:
                mx = max(np.max(np.abs(d1)), np.max(np.abs(d2)))
                ax.set_ylim(-mx * 1.2 - 0.1, mx * 1.2 + 0.1)
        
        self.ax_freq.set_ylim(0, max(30, max(cf.max(), pf.max()) * 1.2))

def main():
    rclpy.init()
    node = Nav2Analyzer()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    try:
        viz = ModernVisualizer(node)
    except KeyboardInterrupt: pass
    finally:
        node.close()
        plt.close('all') 
        if rclpy.ok(): rclpy.shutdown()
        generate_post_run_report(node.csv_filename)

if __name__ == '__main__':
    main()