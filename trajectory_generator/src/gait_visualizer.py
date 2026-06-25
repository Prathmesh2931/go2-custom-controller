import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Slider

# --- ROBOT KINEMATICS ---
HIP_X = 0.1934    # Front/Back offset from CoM
HIP_Y = 0.14      # Left/Right offset (Wide Stance)
STANCE_Z = -0.28  # Target stance height
LIFT_H = 0.08     # Swing clearance

# --- SETUP PLOTS ---
fig = plt.figure(figsize=(18, 7))
fig.canvas.manager.set_window_title('Go2 Interactive Gait & Odometry Kinematics')

# Subplot 1: Top-Down (X-Y) View (Local Robot Frame)
ax_top = plt.subplot(1, 3, 1)
ax_top.set_title("Local Footprint (X-Y)", fontweight='bold')
ax_top.set_xlim(-0.4, 0.4)
ax_top.set_ylim(-0.3, 0.3)
ax_top.set_aspect('equal')
ax_top.grid(True, linestyle='--', alpha=0.6)
ax_top.axhline(0, color='black', lw=1, alpha=0.3)
ax_top.axvline(0, color='black', lw=1, alpha=0.3)
com_marker_top = ax_top.scatter([0], [0], color='black', s=100, marker='X', label='CoM')

# Subplot 2: Side (X-Z) View for Left Front (LF)
ax_side = plt.subplot(1, 3, 2)
ax_side.set_title("LF Swing Arc (X-Z)", fontweight='bold')
ax_side.set_xlim(0.0, 0.4)
ax_side.set_ylim(-0.4, -0.1)
ax_side.set_aspect('equal')
ax_side.grid(True, linestyle='--', alpha=0.6)
ax_side.axhline(STANCE_Z, color='saddlebrown', lw=4, label='Ground')

# Subplot 3: Global Body Path (World Frame)
ax_odom = plt.subplot(1, 3, 3)
ax_odom.set_title("Global Body Path (Odometry)", fontweight='bold')
ax_odom.set_aspect('equal')
ax_odom.grid(True, linestyle='--', alpha=0.6)

# Visual markers for feet (Local)
colors = ['dodgerblue', 'crimson', 'crimson', 'dodgerblue']
labels = ['LF (Diag 1)', 'RF (Diag 2)', 'LH (Diag 2)', 'RH (Diag 1)']
foot_scatters_top = []
for i in range(4):
    scat = ax_top.scatter([], [], color=colors[i], s=150, label=labels[i])
    foot_scatters_top.append(scat)

lf_arc_line, = ax_side.plot([], [], 'o-', color='dodgerblue', lw=3, markersize=10)
ax_top.legend(loc='upper right', fontsize=8)

# Visual markers for Global Odometry
path_line, = ax_odom.plot([], [], 'k-', lw=2, alpha=0.5, label='Trajectory')
body_marker, = ax_odom.plot([], [], 'ro', markersize=8, label='Robot Base')
heading_line, = ax_odom.plot([], [], 'r-', lw=3)
ax_odom.legend(loc='upper left', fontsize=8)

# Diagnostics Text
text_diag = fig.text(0.02, 0.82, "", fontsize=10, fontfamily='monospace', bbox=dict(facecolor='white', alpha=0.8))

# --- SLIDERS FOR CMD_VEL ---
ax_vx = plt.axes([0.15, 0.1, 0.65, 0.03])
ax_wz = plt.axes([0.15, 0.05, 0.65, 0.03])
slider_vx = Slider(ax_vx, 'Forward (Vx)', -0.5, 0.5, valinit=0.0)
slider_wz = Slider(ax_wz, 'Yaw (Wz)', -1.0, 1.0, valinit=0.0)

# State Variables
phase = 0.0
global_x = 0.0
global_y = 0.0
global_yaw = 0.0
path_history_x = []
path_history_y = []
DT = 0.02  # 20ms update rate

def update(frame):
    global phase, global_x, global_y, global_yaw
    
    vx = slider_vx.val
    wz = slider_wz.val
    speed = np.sqrt(vx**2 + wz**2)
    is_moving = speed > 0.05
    
    # 1. UPDATE GLOBAL ODOMETRY KINEMATICS
    global_yaw += wz * DT
    
    # Only integrate position if actually moving
    if is_moving:
        global_x += (vx * np.cos(global_yaw)) * DT
        global_y += (vx * np.sin(global_yaw)) * DT
        
    path_history_x.append(global_x)
    path_history_y.append(global_y)
    
    # Keep trail memory manageable
    if len(path_history_x) > 300:
        path_history_x.pop(0)
        path_history_y.pop(0)
    
    # 2. UPDATE PHASE CLOCK
    if is_moving:
        phase += 0.02 * 1.5  # Cadence
        if phase >= 1.0: phase -= 1.0
    else:
        phase = 0.0 # 4-LEG IDLE OVERRIDE
        
    contacts = [1, 1, 1, 1]
    xs, ys, zs = [0]*4, [0]*4, [0]*4
    
    # 3. CALCULATE DYNAMIC FOOT PLACEMENTS (LOCAL FRAME)
    for i in range(4):
        # Base Hip Positions
        rx = HIP_X if i < 2 else -HIP_X
        ry = HIP_Y if i % 2 == 0 else -HIP_Y
        
        if is_moving:
            # DIFFERENTIAL KINEMATICS FOR YAW
            # Velocity of foot = V_base + Angular_Vel x Radius
            foot_vx = vx - wz * ry  # Outer legs move faster during turn
            foot_vy = wz * rx       # Front/Back legs sweep sideways during turn
            
            step_x = np.clip(foot_vx * 0.4, -0.15, 0.15)
            step_y = np.clip(foot_vy * 0.4, -0.08, 0.08)
            
            p = phase if (i == 0 or i == 3) else ((phase + 0.5) % 1.0)
            
            if p < 0.4:
                contacts[i] = 0 # SWING
                t = p / 0.4
                xs[i] = rx - step_x/2.0 + step_x * (0.5 - 0.5*np.cos(np.pi * t))
                ys[i] = ry - step_y/2.0 + step_y * (0.5 - 0.5*np.cos(np.pi * t))
                zs[i] = STANCE_Z + LIFT_H * np.sin(np.pi * t)
            else:
                contacts[i] = 1 # STANCE
                t = (p - 0.4) / 0.6
                xs[i] = rx + step_x/2.0 - step_x * t
                ys[i] = ry + step_y/2.0 - step_y * t
                zs[i] = STANCE_Z
        else:
            # IDLE: Lock to nominal positions on the ground
            contacts[i] = 1
            xs[i], ys[i], zs[i] = rx, ry, STANCE_Z
            
    # 4. UPDATE PLOT VISUALS
    for i in range(4):
        foot_scatters_top[i].set_offsets(np.c_[xs[i], ys[i]])
        # Fade out legs that are in the air (Swing)
        foot_scatters_top[i].set_alpha(1.0 if contacts[i] == 1 else 0.3)
        
    lf_arc_line.set_data([HIP_X, xs[0]], [0, zs[0]])
    lf_arc_line.set_alpha(1.0 if contacts[0] == 1 else 0.3)
    
    # Update Odometry Plot
    path_line.set_data(path_history_x, path_history_y)
    body_marker.set_data([global_x], [global_y])
    # Draw heading vector (Length 0.2m for visibility)
    hx = global_x + 0.2 * np.cos(global_yaw)
    hy = global_y + 0.2 * np.sin(global_yaw)
    heading_line.set_data([global_x, hx], [global_y, hy])
    
    # Dynamic following camera (1.5m radius around robot)
    ax_odom.set_xlim(global_x - 1.5, global_x + 1.5)
    ax_odom.set_ylim(global_y - 1.5, global_y + 1.5)
    
    # 5. UPDATE DIAGNOSTIC TEXT
    diag_str = f"GLOBAL PHASE: {phase:.2f} | IS_MOVING: {is_moving}\n"
    diag_str += f"GLOBAL POS  : X={global_x:+.2f}, Y={global_y:+.2f}, YAW={np.degrees(global_yaw):+.0f} deg\n\n"
    diag_str += "CONTACT ESTIMATION (0=Swing, 1=Stance):\n"
    diag_str += f"LF (Diag 1) : {contacts[0]}  |  RF (Diag 2) : {contacts[1]}\n"
    diag_str += f"LH (Diag 2) : {contacts[2]}  |  RH (Diag 1) : {contacts[3]}"
    
    text_diag.set_text(diag_str)
    
    return foot_scatters_top + [lf_arc_line, path_line, body_marker, heading_line, text_diag]

ani = animation.FuncAnimation(fig, update, frames=200, interval=20, blit=False)
plt.subplots_adjust(bottom=0.25, left=0.05, right=0.98)
plt.show()