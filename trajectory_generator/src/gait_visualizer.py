import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import threading
from collections import deque

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('gait_visualizer')
        
        # Subscriptions
        self.sub_target = self.create_subscription(
            Float64MultiArray, '/gait/planned_positions', self.target_cb, 10)
        self.sub_actual = self.create_subscription(
            JointState, '/joint_states', self.actual_cb, 10)
            
        # Go2 Kinematics Constants
        self.HIP_OFFSET = 0.0955
        self.THIGH_LEN = 0.213
        self.CALF_LEN = 0.213
        self.hip_offset_x = [0.1934, 0.1934, -0.1934, -0.1934]

        # Joint Names
        self.joint_names = [
            "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint",
            "rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
            "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint",
            "rh_hip_joint", "rh_upper_leg_joint", "rh_lower_leg_joint"
        ]

        # Trajectory History (250Hz * 60 seconds = 15000 points)
        hist_len = 15000
        
        # LF (Leg 0)
        self.target_x_lf = deque(maxlen=hist_len)
        self.target_z_lf = deque(maxlen=hist_len)
        self.actual_x_lf = deque(maxlen=hist_len)
        self.actual_z_lf = deque(maxlen=hist_len)
        
        # RH (Leg 3)
        self.target_x_rh = deque(maxlen=hist_len)
        self.target_z_rh = deque(maxlen=hist_len)
        self.actual_x_rh = deque(maxlen=hist_len)
        self.actual_z_rh = deque(maxlen=hist_len)

    def calc_fk(self, q1, q2, q3, leg_idx):
        l1 = self.HIP_OFFSET if (leg_idx == 0 or leg_idx == 2) else -self.HIP_OFFSET
        l2 = self.THIGH_LEN
        l3 = self.CALF_LEN

        s1, c1 = np.sin(q1), np.cos(q1)
        s2, c2 = np.sin(q2), np.cos(q2)
        s23, c23 = np.sin(q2 + q3), np.cos(q2 + q3)

        # X/Z relative to hip
        x = -l2 * s2 - l3 * s23
        z = l1 * s1 - c1 * (l2 * c2 + l3 * c23)

        # Transform to world relative frame
        x_world = x + self.hip_offset_x[leg_idx]
        return x_world, z

    def target_cb(self, msg):
        if len(msg.data) >= 12:
            # LF (Index 0)
            tx, tz = self.calc_fk(msg.data[0], msg.data[1], msg.data[2], 0)
            self.target_x_lf.append(tx)
            self.target_z_lf.append(tz)
            
            # RH (Index 3)
            tx, tz = self.calc_fk(msg.data[9], msg.data[10], msg.data[11], 3)
            self.target_x_rh.append(tx)
            self.target_z_rh.append(tz)

    def actual_cb(self, msg):
        if len(msg.name) >= 12:
            try:
                # Extract indices safely in case /joint_states scrambles the order
                lf_q1 = msg.position[msg.name.index("lf_hip_joint")]
                lf_q2 = msg.position[msg.name.index("lf_upper_leg_joint")]
                lf_q3 = msg.position[msg.name.index("lf_lower_leg_joint")]
                
                rh_q1 = msg.position[msg.name.index("rh_hip_joint")]
                rh_q2 = msg.position[msg.name.index("rh_upper_leg_joint")]
                rh_q3 = msg.position[msg.name.index("rh_lower_leg_joint")]

                ax, az = self.calc_fk(lf_q1, lf_q2, lf_q3, 0)
                self.actual_x_lf.append(ax)
                self.actual_z_lf.append(az)
                
                ax, az = self.calc_fk(rh_q1, rh_q2, rh_q3, 3)
                self.actual_x_rh.append(ax)
                self.actual_z_rh.append(az)
            except ValueError:
                pass # Still initializing

def run_ros(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = TrajectoryVisualizer()
    
    # Run ROS in a background thread so Matplotlib can own the main thread
    ros_thread = threading.Thread(target=run_ros, args=(node,), daemon=True)
    ros_thread.start()

    # --- Matplotlib Setup ---
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    fig.canvas.manager.set_window_title('Go2 Cartesian PD Tracking Analyzer')

    # Formatting axes
    for ax, title in zip([ax1, ax2], ['Left Front (LF)', 'Right Hind (RH)']):
        ax.set_title(title, fontweight='bold')
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Z Position (m)')
        ax.grid(True, linestyle='--', alpha=0.6)
        # Expanded Z-axis limits in case the physical robot sags heavily
        ax.set_ylim(-0.45, -0.05)  
        
    # Expanded X-axis limits to prevent clipping
    ax1.set_xlim(-0.05, 0.45)   # LF Hip is at +0.1934
    ax2.set_xlim(-0.45, 0.05)  # RH Hip is at -0.1934

    # Target lines (Dashed Blue)
    line_target_lf, = ax1.plot([], [], 'b--', lw=2, label='Target Orbit')
    line_target_rh, = ax2.plot([], [], 'b--', lw=2, label='Target Orbit')
    
    # Actual lines (Solid Red)
    line_actual_lf, = ax1.plot([], [], 'r-', lw=2.5, label='Actual Orbit')
    line_actual_rh, = ax2.plot([], [], 'r-', lw=2.5, label='Actual Orbit')

    # Current position dots
    dot_target_lf, = ax1.plot([], [], 'bo', markersize=8)
    dot_actual_lf, = ax1.plot([], [], 'ro', markersize=8)
    dot_target_rh, = ax2.plot([], [], 'bo', markersize=8)
    dot_actual_rh, = ax2.plot([], [], 'ro', markersize=8)

    ax1.legend(loc='upper right')
    ax2.legend(loc='upper right')

    def animate(i):
        # Update LF
        if len(node.target_x_lf) > 0 and len(node.actual_x_lf) > 0:
            # Thread-safe extraction to prevent broadcast shape errors
            t_len_lf = min(len(node.target_x_lf), len(node.target_z_lf))
            a_len_lf = min(len(node.actual_x_lf), len(node.actual_z_lf))
            
            tx_lf = list(node.target_x_lf)[-t_len_lf:]
            tz_lf = list(node.target_z_lf)[-t_len_lf:]
            ax_lf = list(node.actual_x_lf)[-a_len_lf:]
            az_lf = list(node.actual_z_lf)[-a_len_lf:]

            line_target_lf.set_data(tx_lf, tz_lf)
            dot_target_lf.set_data([tx_lf[-1]], [tz_lf[-1]])
            
            line_actual_lf.set_data(ax_lf, az_lf)
            dot_actual_lf.set_data([ax_lf[-1]], [az_lf[-1]])

        # Update RH
        if len(node.target_x_rh) > 0 and len(node.actual_x_rh) > 0:
            t_len_rh = min(len(node.target_x_rh), len(node.target_z_rh))
            a_len_rh = min(len(node.actual_x_rh), len(node.actual_z_rh))
            
            tx_rh = list(node.target_x_rh)[-t_len_rh:]
            tz_rh = list(node.target_z_rh)[-t_len_rh:]
            ax_rh = list(node.actual_x_rh)[-a_len_rh:]
            az_rh = list(node.actual_z_rh)[-a_len_rh:]

            line_target_rh.set_data(tx_rh, tz_rh)
            dot_target_rh.set_data([tx_rh[-1]], [tz_rh[-1]])
            
            line_actual_rh.set_data(ax_rh, az_rh)
            dot_actual_rh.set_data([ax_rh[-1]], [az_rh[-1]])

        return line_target_lf, line_actual_lf, dot_target_lf, dot_actual_lf, \
               line_target_rh, line_actual_rh, dot_target_rh, dot_actual_rh

    ani = animation.FuncAnimation(fig, animate, interval=20, blit=True)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()