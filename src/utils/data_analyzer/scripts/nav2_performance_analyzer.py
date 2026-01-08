#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import threading
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import matplotlib.gridspec as gridspec
import sys
import platform
import csv
import os

# --- Dependency Check ---
try:
    from ros_interfaces.msg import MpcPositionCommand
except ImportError:
    print("Error: Could not import ros_interfaces.msg.MpcPositionCommand")
    sys.exit(1)

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

# --- Theme Configuration (Modern Dark UI) ---
THEME = {
    'bg': '#1e1e2e',          # Dark Blue-Grey Background
    'plot_bg': '#252535',     # Plot Area Background
    'text': '#cdd6f4',        # Soft White Text
    'text_dim': '#6c7086',    # Dimmed Text
    'grid': '#45475a',        # Grid Lines
    'pos': '#f38ba8',         # Red (Position)
    'vel': '#fab387',         # Orange (Velocity)
    'acc': '#f9e2af',         # Yellow (Acceleration)
    'plan': '#89b4fa',        # Blue (Plan)
    'freq': '#a6e3a1',        # Green (Frequency)
    'ok': '#a6e3a1',          # Green (Status OK)
    'err': '#f38ba8',         # Red (Status Error)
    'btn': '#313244',         # Button Background
    'btn_hover': '#45475a',   # Button Hover
    'fill_alpha': 0.2         # Fill Opacity
}

# --- Configuration Parameters ---
TOPIC_OPT_PATH = '/opt_path'
TOPIC_ODOM = '/aft_mapped_to_init'
TOPIC_IMU = '/livox/imu'
TOPIC_CMD_VEL = '/cmd_vel'

# Optimized for smoothness (High FPS)
MAX_HISTORY = 1200      # Increased buffer to keep ~40s window at higher rate
UPDATE_INTERVAL_MS = 33 # ~30 FPS for smooth animation
TIMEOUT_SEC = 2.0
RECORD_RATE_HZ = 30.0   # 30Hz Sampling

class Nav2Analyzer(Node):
    def __init__(self):
        super().__init__('nav2_performance_analyzer')
        self.lock = threading.Lock()
        self.running = True
        
        # Data Containers
        self.times = []
        self.pos_errors = []
        self.vel_errors = []
        self.acc_errors = []
        self.planner_freqs = [] 
        self.control_freqs = []
        
        # Topic Heartbeats (Last received time)
        self.topic_status = {
            'Plan': 0.0,
            'Odom': 0.0,
            'IMU': 0.0,
            'Cmd': 0.0
        }
        
        # State
        self.latest_plan = None
        self.latest_odom = None 
        self.latest_imu_acc = 0.0
        
        # Frequency Calculation Counters
        self.cmd_vel_count = 0
        self.plan_count = 0 
        self.last_freq_calc_time = time.time()
        
        self.current_ctrl_freq = 0.0
        self.current_plan_freq = 0.0 
        
        self.start_time = time.time()

        # Subscribers
        self.create_subscription(MpcPositionCommand, TOPIC_OPT_PATH, self.plan_cb, 1)
        self.create_subscription(Odometry, TOPIC_ODOM, self.odom_cb, 10)
        self.create_subscription(Imu, TOPIC_IMU, self.imu_cb, 10)
        self.create_subscription(Twist, TOPIC_CMD_VEL, self.cmd_vel_cb, 10)
        
        # Create a separate timer for data recording
        self.create_timer(1.0 / RECORD_RATE_HZ, self.data_recording_loop)

        self.get_logger().info("Nav2 Analyzer Started (Full Metrics Mode)")

    def plan_cb(self, msg):
        with self.lock:
            self.latest_plan = msg
            self.plan_count += 1 
            self.topic_status['Plan'] = time.time()

    def imu_cb(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        self.latest_imu_acc = np.hypot(ax, ay)
        self.topic_status['IMU'] = time.time()

    def cmd_vel_cb(self, msg):
        with self.lock:
            self.cmd_vel_count += 1
            self.topic_status['Cmd'] = time.time()

    def odom_cb(self, msg):
        with self.lock:
            self.latest_odom = msg
            self.topic_status['Odom'] = time.time()

    def data_recording_loop(self):
        """Independent loop to calculate stats and record data"""
        current_time = time.time()
        
        with self.lock:
            # 1. Frequency Calculation
            dt_freq = current_time - self.last_freq_calc_time
            if dt_freq >= 1.0:
                self.current_ctrl_freq = self.cmd_vel_count / dt_freq
                self.current_plan_freq = self.plan_count / dt_freq
                self.cmd_vel_count = 0
                self.plan_count = 0
                self.last_freq_calc_time = current_time

            # 2. Error Calculation
            pos_err = 0.0
            vel_err = 0.0
            acc_err = 0.0
            
            # Only calculate errors if we have both Plan and Odom
            if self.latest_odom is not None and \
               self.latest_plan is not None and \
               len(self.latest_plan.cmds) > 0:
                
                # Extract Odom
                rx = self.latest_odom.pose.pose.position.x
                ry = self.latest_odom.pose.pose.position.y
                rvx = self.latest_odom.twist.twist.linear.x
                rvy = self.latest_odom.twist.twist.linear.y
                
                # Closest Point Logic
                min_dist = float('inf')
                best_cmd = self.latest_plan.cmds[0]
                
                for cmd in self.latest_plan.cmds:
                    dist = np.hypot(cmd.position.x - rx, cmd.position.y - ry)
                    if dist < min_dist:
                        min_dist = dist
                        best_cmd = cmd
                
                # Calculate Values
                pos_err = min_dist
                ref_vel = np.hypot(best_cmd.velocity.x, best_cmd.velocity.y)
                act_vel = np.hypot(rvx, rvy)
                vel_err = abs(ref_vel - act_vel)
                
                ref_acc = np.hypot(best_cmd.acceleration.x, best_cmd.acceleration.y)
                acc_err = abs(ref_acc - self.latest_imu_acc)

            # 3. Store Data
            rt = current_time - self.start_time
            self.times.append(rt)
            self.pos_errors.append(pos_err)
            self.vel_errors.append(vel_err)
            self.acc_errors.append(acc_err)
            self.planner_freqs.append(self.current_plan_freq)
            self.control_freqs.append(self.current_ctrl_freq)
            
            # Maintain History Limit
            if len(self.times) > MAX_HISTORY:
                for lst in [self.times, self.pos_errors, self.vel_errors, self.acc_errors, self.planner_freqs, self.control_freqs]:
                    lst.pop(0)

    def get_topic_health(self):
        """Check if topics are active"""
        now = time.time()
        health = {}
        for topic, last_t in self.topic_status.items():
            health[topic] = (now - last_t) < TIMEOUT_SEC
        return health

class ModernVisualizer:
    def __init__(self, node):
        self.node = node
        self.paused = False

        # --- Font Configuration ---
        plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'sans-serif']
        plt.rcParams['axes.unicode_minus'] = False
        plt.rcParams['toolbar'] = 'None' 

        # --- Plot Initialization ---
        self.fig = plt.figure(figsize=(12, 11), facecolor=THEME['bg'])
        self.fig.canvas.manager.set_window_title('NAV2 Minco Performance Analyzer')
        self.fig.canvas.mpl_connect('close_event', self.on_close)

        # Layout: Header (0.8), 4 Plots (1.0 each)
        gs = gridspec.GridSpec(5, 1, height_ratios=[0.8, 1, 1, 1, 1], hspace=0.45, top=0.96, bottom=0.07, left=0.08, right=0.92)

        # 1. Dashboard Header (HUD)
        self.ax_header = self.fig.add_subplot(gs[0])
        self.ax_header.axis('off')
        
        # -- Title & Global Status --
        self.txt_title = self.ax_header.text(0.0, 0.9, "NAV2 PERFORMANCE MONITOR", color=THEME['text'], fontsize=14, fontweight='bold', ha='left')
        self.txt_time = self.ax_header.text(1.0, 0.9, "T: 00:00", color=THEME['text_dim'], fontsize=12, ha='right', family='monospace')

        # -- Topic Status Indicators --
        self.topic_indicators = {}
        topics = ['Plan', 'Odom', 'IMU', 'Cmd']
        start_x = 0.0
        gap_x = 0.25
        
        for i, topic in enumerate(topics):
            x = start_x + i * gap_x
            dot = self.ax_header.text(x, 0.65, "●", color=THEME['err'], fontsize=16, ha='left')
            lbl = self.ax_header.text(x + 0.03, 0.65, topic, color=THEME['text_dim'], fontsize=10, ha='left', va='center')
            self.topic_indicators[topic] = dot
        
        # -- Big Stats Display (Updated for 4 metrics) --
        self.stat_pos = self._create_stat_text(0.12, "Pos Error (m)", THEME['pos'])
        self.stat_vel = self._create_stat_text(0.37, "Vel Error (m/s)", THEME['vel'])
        self.stat_acc = self._create_stat_text(0.62, "Acc Error (m/s²)", THEME['acc'])
        self.stat_freq = self._create_stat_text(0.87, "Ctrl Freq (Hz)", THEME['freq'])

        # 2. Plot Areas
        self.axes = []
        self.lines = {}
        self.fills = {}

        # Plot 1: Position Error
        ax1 = self.fig.add_subplot(gs[1], facecolor=THEME['plot_bg'])
        self._style_axis(ax1, "Position Tracking Error", "Error (m)")
        ln1, = ax1.plot([], [], color=THEME['pos'], lw=2)
        fill1 = ax1.fill_between([], [], color=THEME['pos'], alpha=THEME['fill_alpha'])
        self.axes.append(ax1)
        self.lines['pos'] = ln1
        self.fills['pos'] = fill1

        # Plot 2: Velocity Error
        ax2 = self.fig.add_subplot(gs[2], facecolor=THEME['plot_bg'], sharex=ax1)
        self._style_axis(ax2, "Velocity Tracking Error", "Error (m/s)")
        ln2, = ax2.plot([], [], color=THEME['vel'], lw=2)
        fill2 = ax2.fill_between([], [], color=THEME['vel'], alpha=THEME['fill_alpha'])
        self.axes.append(ax2)
        self.lines['vel'] = ln2
        self.fills['vel'] = fill2

        # Plot 3: Acceleration Error (Restored!)
        ax3 = self.fig.add_subplot(gs[3], facecolor=THEME['plot_bg'], sharex=ax1)
        self._style_axis(ax3, "Acceleration Tracking Error", "Error (m/s²)")
        ln3, = ax3.plot([], [], color=THEME['acc'], lw=2)
        fill3 = ax3.fill_between([], [], color=THEME['acc'], alpha=THEME['fill_alpha'])
        self.axes.append(ax3)
        self.lines['acc'] = ln3
        self.fills['acc'] = fill3

        # Plot 4: System Frequency
        ax4 = self.fig.add_subplot(gs[4], facecolor=THEME['plot_bg'], sharex=ax1)
        self._style_axis(ax4, "System Frequencies (Plan vs Ctrl)", "Frequency (Hz)")
        
        ln4_c, = ax4.plot([], [], color=THEME['freq'], lw=2, linestyle='-', alpha=0.9, label='Ctrl Freq')
        ln4_p, = ax4.plot([], [], color=THEME['plan'], lw=2, linestyle='--', alpha=1.0, zorder=10, label='Plan Freq')
        fill4 = ax4.fill_between([], [], color=THEME['freq'], alpha=0.1)
        
        ax4.legend([ln4_p, ln4_c], ['Plan Freq', 'Ctrl Freq'], loc='upper left', frameon=False, labelcolor=THEME['text'])
        
        self.axes.append(ax4)
        self.lines['plan'] = ln4_p
        self.lines['freq'] = ln4_c
        self.fills['freq'] = fill4

        # Input Events
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

        # Buttons
        ax_save = plt.axes([0.65, 0.02, 0.1, 0.04])
        self.btn_save = Button(ax_save, 'Save IMG', color=THEME['btn'], hovercolor=THEME['btn_hover'])
        self.btn_save.label.set_color(THEME['text'])
        self.btn_save.on_clicked(self.save_screenshot)

        ax_csv = plt.axes([0.50, 0.02, 0.1, 0.04])
        self.btn_csv = Button(ax_csv, 'Save CSV', color=THEME['btn'], hovercolor=THEME['btn_hover'])
        self.btn_csv.label.set_color(THEME['text'])
        self.btn_csv.on_clicked(self.save_csv)

        ax_pause = plt.axes([0.80, 0.02, 0.1, 0.04])
        self.btn_pause = Button(ax_pause, 'Pause', color=THEME['btn'], hovercolor=THEME['btn_hover'])
        self.btn_pause.label.set_color(THEME['text'])
        self.btn_pause.on_clicked(self.toggle_pause)

        self.ani = FuncAnimation(self.fig, self.update, interval=UPDATE_INTERVAL_MS, blit=False)
        plt.show()

    def on_close(self, event):
        self.node.running = False
        plt.close(self.fig)

    def _create_stat_text(self, x, label, color):
        self.ax_header.text(x, 0.35, label, color=THEME['text_dim'], fontsize=10, ha='center')
        text_obj = self.ax_header.text(x, 0.1, "0.000", color=color, fontsize=20, fontweight='bold', ha='center', family='monospace')
        return text_obj

    def _style_axis(self, ax, title, ylabel, is_right=False):
        ax.set_title(title, color=THEME['text'], loc='left', fontsize=10, pad=5)
        ax.set_ylabel(ylabel, color=THEME['text_dim'], fontsize=9)
        ax.tick_params(axis='x', colors=THEME['text_dim'], labelsize=8)
        ax.tick_params(axis='y', colors=THEME['text_dim'], labelsize=8)
        
        for spine in ax.spines.values():
            spine.set_visible(False)
        ax.spines['bottom'].set_visible(True)
        ax.spines['bottom'].set_color(THEME['grid'])
        ax.grid(True, color=THEME['grid'], linestyle=':', linewidth=0.5, alpha=0.5)
        
        if is_right: ax.yaxis.set_label_position("right")

    def toggle_pause(self, event):
        self.paused = not self.paused
        print(f"UI {'PAUSED' if self.paused else 'RESUMED'}")

    def save_screenshot(self, event):
        filename = f"nav2_analysis_{int(time.time())}.png"
        self.fig.savefig(filename, facecolor=THEME['bg'])
        print(f"Screenshot saved: {filename}")

    def save_csv(self, event):
        filename = f"nav2_data_{int(time.time())}.csv"
        try:
            with self.node.lock:
                if len(self.node.times) == 0:
                    print("No data to save!")
                    return
                rows = zip(self.node.times, self.node.pos_errors, self.node.vel_errors, self.node.acc_errors, self.node.planner_freqs, self.node.control_freqs)
            
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Time', 'Pos_Error', 'Vel_Error', 'Acc_Error', 'Plan_Freq', 'Ctrl_Freq'])
                writer.writerows(rows)
            print(f"Data saved to {filename}")
        except Exception as e:
            print(f"Error saving CSV: {e}")

    def on_key(self, event):
        if event.key == ' ': self.toggle_pause(None)
        elif event.key == 's': self.save_screenshot(None)

    def update(self, frame):
        if not self.node.running: return
        if self.paused: return

        # 1. Update Indicators
        health = self.node.get_topic_health()
        for topic, is_ok in health.items():
            if topic in self.topic_indicators:
                self.topic_indicators[topic].set_color(THEME['ok'] if is_ok else THEME['err'])

        with self.node.lock:
            if not self.node.times: return
            times = np.array(self.node.times)
            pos = np.array(self.node.pos_errors)
            vel = np.array(self.node.vel_errors)
            acc = np.array(self.node.acc_errors)
            plan_freqs = np.array(self.node.planner_freqs)
            ctrl_freqs = np.array(self.node.control_freqs)

        # 2. Update Stats
        self.txt_time.set_text(f"T: {times[-1]:.1f}s")
        self.stat_pos.set_text(f"{pos[-1]:.4f}")
        self.stat_vel.set_text(f"{vel[-1]:.3f}")
        self.stat_acc.set_text(f"{acc[-1]:.3f}")
        self.stat_freq.set_text(f"{ctrl_freqs[-1]:.1f}")

        # 3. Update Lines
        self.lines['pos'].set_data(times, pos)
        self.lines['vel'].set_data(times, vel)
        self.lines['acc'].set_data(times, acc)
        self.lines['plan'].set_data(times, plan_freqs)
        self.lines['freq'].set_data(times, ctrl_freqs)

        # 4. Update Fills
        try:
            self.fills['pos'].remove()
            self.fills['vel'].remove()
            self.fills['acc'].remove()
            self.fills['freq'].remove()
        except: pass 
        
        self.fills['pos'] = self.axes[0].fill_between(times, 0, pos, color=THEME['pos'], alpha=THEME['fill_alpha'])
        self.fills['vel'] = self.axes[1].fill_between(times, 0, vel, color=THEME['vel'], alpha=THEME['fill_alpha'])
        self.fills['acc'] = self.axes[2].fill_between(times, 0, acc, color=THEME['acc'], alpha=THEME['fill_alpha'])
        self.fills['freq'] = self.axes[3].fill_between(times, 0, ctrl_freqs, color=THEME['freq'], alpha=0.1)

        # 5. Dynamic Scaling
        if len(times) > 1:
            xmin, xmax = times.min(), times.max()
            span = xmax - xmin
            if span < 5: xmin = xmax - 5 
            
            for ax in self.axes:
                ax.set_xlim(xmin, xmax)
            
            self.axes[0].set_ylim(0, pos.max() * 1.2 + 0.01)
            self.axes[1].set_ylim(0, vel.max() * 1.2 + 0.01)
            self.axes[2].set_ylim(0, acc.max() * 1.2 + 0.01)
            
            max_f = max(plan_freqs.max(), ctrl_freqs.max()) if len(plan_freqs) > 0 else 20.0
            self.axes[3].set_ylim(0, max_f * 1.3 + 1.0)

def main():
    rclpy.init()
    node = Nav2Analyzer()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    try:
        viz = ModernVisualizer(node)
    except KeyboardInterrupt: pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()