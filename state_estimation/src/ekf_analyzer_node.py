#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np
import matplotlib.pyplot as plt

class EkfAnalyzerNode(Node):
    def __init__(self):
        super().__init__('ekf_analyzer_node')
        
        self.sub_est_ = self.create_subscription(Odometry, 'odom/estimate', self.cb_est, 10)
        self.sub_gt_ = self.create_subscription(Odometry, '/odom/ground_truth', self.cb_gt, 10)
        self.sub_diag_ = self.create_subscription(Float64MultiArray, 'ekf/diagnostics', self.cb_diag, 10)
        
        # In-memory storage buckets
        self.time_history = []
        self.error_vx = []
        self.sigma_vx = []
        self.nis_history = []
        
        self.latest_gt_vx = 0.0
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info("Live EKF Performance Collector Active. Press Ctrl+C to render profiles.")

    def cb_gt(self, msg):
        self.latest_gt_vx = msg.twist.twist.linear.x

    def cb_est(self, msg):
        current_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        self.time_history.append(current_time)
        # Calculate active tracking residual delta
        self.error_vx.append(msg.twist.twist.linear.x - self.latest_gt_vx)

    def cb_diag(self, msg):
        if len(msg.data) >= 7:
            # data[0] is P_(3,3) variance -> convert to standard deviation bounds
            self.sigma_vx.append(np.sqrt(msg.data[0]))
            self.nis_history.append(msg.data[6])

    def destroy_node(self):
        # Trigger full graphical data analysis pipeline automatically on clean exit
        if len(self.time_history) > 10:
            self.render_plots()
        super().destroy_node()

    def render_plots(self):
        min_len = min(len(self.time_history), len(self.error_vx), len(self.sigma_vx), len(self.nis_history))
        t = self.time_history[:min_len]
        err = self.error_vx[:min_len]
        sd = self.sigma_vx[:min_len]
        nis = self.nis_history[:min_len]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7))
        
        # Subplot 1: 3-Sigma Bounding Envelope
        ax1.plot(t, err, label='Estimation Error ($e_{vx}$)', color='crimson', lw=1.5)
        ax1.plot(t, 3 * np.array(sd), color='navy', linestyle='--', label='+3$\\sigma$ Covariance Boundary')
        ax1.plot(t, -3 * np.array(sd), color='navy', linestyle='--')
        ax1.set_title('EKF Velocity Consistency Profile ($3\\sigma$ Envelopes)')
        ax1.set_ylabel('Error [m/s]')
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='upper right')

        # Subplot 2: Innovation NIS Chi-Squared Distribution Line
        ax2.plot(t, nis, color='purple', alpha=0.5, label='Empirical Innovation NIS ($\\epsilon_N$)')
        ax2.axhline(y=3.0, color='darkgreen', linestyle='-', label='Theoretical Expected Value (DOF=3)', lw=2)
        ax2.axhline(y=7.81, color='black', linestyle=':', label='95% Critical Limit Gating Boundary (7.81)')
        ax2.set_title(f'Normalized Innovation Squared Tracker | Mean: {np.mean(nis):.3f}')
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('NIS Score')
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc='upper right')

        plt.tight_layout()
        plt.show()

def main(argc=None):
    rclpy.init(args=argc)
    node = EkfAnalyzerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()