import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
import threading
import collections
import time

# Massive buffer to hold ~60 seconds of high-frequency (250Hz) data
HISTORY_LEN = 15000

class DataContainer:
    def __init__(self):
        self.lock = threading.Lock()
        self.t_start = None
        self.is_paused = False
        
        # CoM Tracking
        self.com_x = collections.deque(maxlen=HISTORY_LEN)
        self.com_z = collections.deque(maxlen=HISTORY_LEN)
        
        # Time axis
        self.times = collections.deque(maxlen=HISTORY_LEN)
        
        # LF Joints
        self.lf_hip = collections.deque(maxlen=HISTORY_LEN)
        self.lf_thigh = collections.deque(maxlen=HISTORY_LEN)
        self.lf_calf = collections.deque(maxlen=HISTORY_LEN)
        
        # LH Joints
        self.lh_hip = collections.deque(maxlen=HISTORY_LEN)
        self.lh_thigh = collections.deque(maxlen=HISTORY_LEN)
        self.lh_calf = collections.deque(maxlen=HISTORY_LEN)

class GaitAnalyzerNode(Node):
    def __init__(self, data_container):
        super().__init__('gait_analyzer')
        self.data = data_container
        
        self.sub_odom = self.create_subscription(
            Odometry, '/odom/ground_truth', self.odom_callback, 10)
            
        self.sub_js = self.create_subscription(
            JointState, '/joint_states', self.js_callback, 10)
            
        self.get_logger().info("Gait Analyzer Subscribed to Topics... 60s Buffer Active.")

    def odom_callback(self, msg):
        # Stop recording new data if the user paused the graph to inspect
        if self.data.is_paused:
            return
            
        with self.data.lock:
            self.data.com_x.append(msg.pose.pose.position.x)
            self.data.com_z.append(msg.pose.pose.position.z)

    def js_callback(self, msg):
        if self.data.is_paused:
            return
            
        names = msg.name
        positions = msg.position
        
        try:
            # Find the indices for LF and LH legs
            i_lf_h = names.index("lf_hip_joint")
            i_lf_t = names.index("lf_upper_leg_joint")
            i_lf_c = names.index("lf_lower_leg_joint")
            
            i_lh_h = names.index("lh_hip_joint")
            i_lh_t = names.index("lh_upper_leg_joint")
            i_lh_c = names.index("lh_lower_leg_joint")
            
            with self.data.lock:
                if self.data.t_start is None:
                    self.data.t_start = time.time()
                
                t_elapsed = time.time() - self.data.t_start
                self.data.times.append(t_elapsed)
                
                self.data.lf_hip.append(positions[i_lf_h])
                self.data.lf_thigh.append(positions[i_lf_t])
                self.data.lf_calf.append(positions[i_lf_c])
                
                self.data.lh_hip.append(positions[i_lh_h])
                self.data.lh_thigh.append(positions[i_lh_t])
                self.data.lh_calf.append(positions[i_lh_c])
                
        except ValueError:
            pass # Ignore messages that don't have all joint names

def main(args=None):
    rclpy.init(args=args)
    data = DataContainer()
    node = GaitAnalyzerNode(data)
    
    # Run ROS 2 in a background thread so Matplotlib can own the main thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()
    
    # --- MATPLOTLIB SETUP ---
    fig = plt.figure(figsize=(14, 9))
    fig.canvas.manager.set_window_title('Go2 Gait Diagnostics - Live 60s Buffer')
    
    # 1. CoM Trajectory (X vs Z)
    ax_com = plt.subplot(3, 1, 1)
    ax_com.set_title("CoM Trajectory (Forward X vs Height Z)", fontweight='bold')
    ax_com.set_ylabel("Height (Z) [m]")
    ax_com.grid(True, linestyle='--')
    line_com, = ax_com.plot([], [], 'k-', lw=2, label="CoM Path")
    ax_com.legend(loc='upper right')
    
    # 2. LF Joint Trajectories vs Time
    ax_lf = plt.subplot(3, 1, 2)
    ax_lf.set_title("Front-Left (LF) Joints", fontweight='bold')
    ax_lf.set_ylabel("Angle [rad]")
    ax_lf.grid(True, linestyle='--')
    line_lf_h, = ax_lf.plot([], [], 'r-', label="LF Hip")
    line_lf_t, = ax_lf.plot([], [], 'g-', label="LF Thigh")
    line_lf_c, = ax_lf.plot([], [], 'b-', label="LF Calf")
    ax_lf.legend(loc='upper right')
    
    # 3. LH Joint Trajectories vs Time
    ax_lh = plt.subplot(3, 1, 3)
    ax_lh.set_title("Hind-Left (LH) Joints", fontweight='bold')
    ax_lh.set_xlabel("Time [s]")
    ax_lh.set_ylabel("Angle [rad]")
    ax_lh.grid(True, linestyle='--')
    line_lh_h, = ax_lh.plot([], [], 'r--', label="LH Hip")
    line_lh_t, = ax_lh.plot([], [], 'g--', label="LH Thigh")
    line_lh_c, = ax_lh.plot([], [], 'b--', label="LH Calf")
    ax_lh.legend(loc='upper right')

    plt.tight_layout()
    plt.subplots_adjust(bottom=0.15) # Make room for the button

    # --- PAUSE BUTTON ---
    ax_pause = plt.axes([0.81, 0.03, 0.15, 0.06])
    btn_pause = Button(ax_pause, 'Pause Graph')

    def toggle_pause(event):
        data.is_paused = not data.is_paused
        if data.is_paused:
            btn_pause.label.set_text('Resume Graph')
            # Set the color to visually indicate paused state
            btn_pause.color = 'lightcoral'
        else:
            btn_pause.label.set_text('Pause Graph')
            btn_pause.color = '0.85'

    btn_pause.on_clicked(toggle_pause)

    def update(frame):
        # Do not modify the plot axis limits if paused, 
        # so the user can freely pan and zoom around the frozen data!
        if data.is_paused:
            return [line_com, line_lf_h, line_lf_t, line_lf_c, line_lh_h, line_lh_t, line_lh_c]
            
        with data.lock:
            if len(data.times) == 0:
                return [line_com, line_lf_h, line_lf_t, line_lf_c, line_lh_h, line_lh_t, line_lh_c]
            
            # Copy data for thread safety while plotting
            times = list(data.times)
            cx = list(data.com_x)
            cz = list(data.com_z)
            
            lf_h = list(data.lf_hip)
            lf_t = list(data.lf_thigh)
            lf_c = list(data.lf_calf)
            
            lh_h = list(data.lh_hip)
            lh_t = list(data.lh_thigh)
            lh_c = list(data.lh_calf)

        # Update CoM Plot
        if len(cx) > 0 and len(cz) > 0:
            line_com.set_data(cx, cz)
            # Add padding to view window
            ax_com.set_xlim(min(cx) - 0.05, max(cx) + 0.05)
            # Center around physical Z limits (-0.2 to 0.4)
            ax_com.set_ylim(min(cz) - 0.05, max(cz) + 0.05)

        # Update LF Plot
        line_lf_h.set_data(times, lf_h)
        line_lf_t.set_data(times, lf_t)
        line_lf_c.set_data(times, lf_c)
        # Keep a rolling window or expand
        ax_lf.set_xlim(times[0], times[-1] + 0.1)
        ax_lf.set_ylim(-3.0, 1.5)

        # Update LH Plot
        line_lh_h.set_data(times, lh_h)
        line_lh_t.set_data(times, lh_t)
        line_lh_c.set_data(times, lh_c)
        ax_lh.set_xlim(times[0], times[-1] + 0.1)
        ax_lh.set_ylim(-3.0, 1.5)

        return [line_com, line_lf_h, line_lf_t, line_lf_c, line_lh_h, line_lh_t, line_lh_c]

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()