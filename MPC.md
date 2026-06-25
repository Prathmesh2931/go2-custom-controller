1. Trap A: The Lateral Sprawl Singularity

Look at your hip positions: lf_hip_joint is stuck at +1.01 rad and rf_hip_joint is stuck at -1.01 rad. Because the hip limits are capped at ±1.0472 rad, all four of your hip links are maxed out, splaying the legs wide open like a frog.

When the chassis is lying flat on its stomach, the large trunk box creates massive surface contact friction against the floor. Because the hips are splayed out laterally at nearly 60 degrees, the leg mechanisms have lost all vertical leverage. The motors are applying absolute maximum torque (-23.70 Nm and 45.43 Nm), but instead of driving the feet downward to lift the chassis, they are jamming the link joints sideways against the floor constraints.

2. rap B: The Discrete Bang-Bang Limit Cycle

Look at your velocity profiles and output torques across sequential timesteps:

    Frame 1: lf_lower_leg_joint Velocity = 22.52 rad/s | Torque = -45.43 Nm

    Frame 2: lf_lower_leg_joint Velocity = -25.26 rad/s | Torque = 45.43 Nm

Your controller is trapped in an infinite numerical limit cycle. Because your ROS 2 control loop operates at a discrete 250 Hz rate, there is a 4ms execution lag between sensor reading and force distribution.

The high tracking gains (Kp​=120) accelerate the light, low-inertia simulation links so quickly that they overshoot the target within a single 4ms timestep. The link strikes the ground constraint, the controller panics, delivers absolute maximum opposite torque, overshoots again, and chatters back and forth endlessly. The robot never stands because it is constantly exhausting its torque headroom fighting its own high-frequency vibrations.

3. # Quadruped Go2 Stand Controller Debugging Log

This log documents the transition from implicit position control to pure explicit torque control on the Unitree Go2 quadruped platform in Gazebo ROS2. It details the infrastructure corrections, control loop anomalies, and multi-body physics deadlocks encountered during the debugging phase.

---

## 1. Architectural Changes & Interface Synchronization

| Target Component | Previous State (Position Control) | Upgraded State (Explicit Effort Control) | Real-World Impact / Root Cause |
| --- | --- | --- | --- |
| **ROS2 Controller Plugin** | `joint_trajectory_controller/JointTrajectoryController` | `effort_controllers/JointGroupEffortController` | Stripped implicit solver safety nets to enable raw torque passthrough. |
| **Command Interface Topic** | `/joint_group_effort_controller/joint_trajectory` | `/joint_group_effort_controller/commands` | Shifted from complex multi-point trajectory messages to flat `Float64MultiArray` torque command vectors. |
| **Hardware Export Pool** | Splitting `<ros2_control>` tags across individual leg xacros. | Consolidated single unified `<ros2_control>` system interface macro in `robot.xacro`. | Prevented the resource manager from crashing due to joint allocation conflicts over separated systems. |
| **Torque Threshold Limit** | Calf ceiling restricted to `35.55 Nm` inside `const.xacro`. | Boosted calf ceiling limit to `45.43 Nm` inside `const.xacro`. | Provided the lower leg joints with the torque headroom needed to pull the model out of compressed singularities. |

---

## 2. Chronological Debugging Log & Core Failure Modes

### Milestone 1: The Hanging Pendulum Gravity Misconception

* **Symptom:** In effort mode, the legs completely collapsed, stalling flat on the ground.
* **Tackled Problem:** The initial feedforward gravity model assumed an open-chain manipulator hanging weightless in space. It completely neglected the inverted static load of the main chassis box pressing down on the leg links.
* **Fix Applied:** Dropped the hanging model. Re-derived feedforward assumptions to project the vertical ground reaction force vector $F_z = \frac{m_{\text{total}} \cdot g}{4}$ up through the geometric stance linkage chain.

### Milestone 2: High-Frequency Limit Cycles & Aliasing

* **Symptom:** Insane joint chatter velocities ($\pm 25\text{ rad/s}$), with output torques banging aggressively between extreme limits ($\pm 45.43\text{ Nm}$) every single frame.
* **Tackled Problem:** High proportional tracking gains ($K_p = 120$) operating over a discrete 4ms loop sampling window ($250\text{ Hz}$). The low-inertia links accelerated so fast that they blew past target profiles before the next sensor state evaluation callback ran.
* **Fix Applied:** Implemented an exponential low-pass torque filter matrix:

$$\tau_{\text{command}} = \tau_{\text{prev}} + \alpha(\tau_{\text{raw}} - \tau_{\text{prev}})$$



Setting $\alpha = 0.25$ completely suppressed high-frequency chattering waves beneath the loop's Nyquist frequency.

### Milestone 3: The Cross-Coupled Pitch Sign Conflict

* **Symptom:** The front pair extended cleanly, but the model pitched backward into a severe 24° nose-up slant and slid out of control.
* **Tackled Problem:** Discovered a coordinate sign inversion conflict between the link geometry and the IMU posture loop data. When the body tilted back, the tracking logic subtracted torque from the rear calves, weakening the legs right as the weight shifted onto them.
* **Fix Applied:** Re-mapped coordinate direction assignments based on kinematic layout. Split the pitch corrections to properly reflect joint mechanics: a positive pitch tilt error adds negative torque to thighs while injecting positive torque into calves.

### Milestone 4: The Post-Mule-Kick Integral Windup

* **Symptom:** The front legs rose smoothly, but the rear legs stayed pinned to the floor before suddenly launching into a violent flip that face-planted the robot's chin into the ground.
* **Tackled Problem:** Because the hindquarters were pinned down by floor traction and weight shift, the rear legs lagged behind. Their error integral pools accumulated massive charge values over the 5-second trajectory rollout window. Breaking static friction discharged this stored energy all at once.
* **Fix Applied:** Implemented a **Trajectory-Gated Integration Profile**. The integral error registers are held strictly at zero while the trajectory sweeps from $t = 0$ to $90\%$. Integration memory is only unlocked at full stance, eliminating rollout windup explosions.

### Milestone 5: Lateral Roll-Bifurcation Capsize

* **Symptom:** The model broke out of its initial launch states but rolled over sideways onto its flank, pinning hip joints to extreme limits.
* **Tackled Problem:** Cross-coupled roll control via thighs/calves caused the links to warp inward under load, narrowing the lateral support polygon. Additionally, a $2.5\text{s}$ hip alignment window was too fast to overcome ground pad friction ($\mu = 0.6$), forcing Stage 2 to initialize with splayed limbs.
* **Fix Applied:** Fully decoupled the 3-axis control layout. Dedicated pitch control exclusively to thighs and calves, while assigning roll corrections directly to hip roll joints to handle left-to-right balancing. Expanded Stage 1 duration to $3.5\text{s}$ to guarantee absolute hip convergence.

---

## 3. Verified Joint Target Calibration Map

> **Important Setup Note:** Always cross-check the initial joint state arrays using `ros2 topic echo /joint_states` against manufacturer values before running raw efforts.

```yaml
# Target 3-Axis posturing constants verified for level simulation standing
Stance Targets:
  q_stand_hip:        0.00   # Rad (Abduction/Adduction track inline under body)
  q_stand_upper_leg:  0.67   # Rad (Thigh link forward lean pitch)
  q_stand_lower_leg: -1.30   # Rad (Calf link backward compression pitch)

```

I am glad we finally got the Go2 standing perfectly!

Since you are writing this up for your documentation (`.md` file), here is a clean, on-point summary of the 5 major roadblocks we hit, the underlying engineering physics behind them, and exactly how we solved them.

You can copy and paste this directly into your notes:

---

Problem faced during the stand pose using effort controller 
## 1. Instantaneous Flipping / Launching ("Initial Condition Explosion")

* **The Symptom:** The moment the controller engaged, the robot violently kicked the floor and launched itself upside down in less than 2 seconds.
* **The Root Cause:** The robot spawns crumpled on the floor. Applying a high-stiffness ($K_p$) target of `0.67` and `-1.30` instantly creates a massive mathematical error step. The PD loop reacted by commanding maximum torque in the first 4 milliseconds. This sudden, explosive impulse force against the Gazebo rigid ground plane caused a violent reaction.
* **The Solution:** Implemented a **Smooth Trajectory Ramp**. By capturing the robot's resting position on tick 1 and interpolating to the final standing target over a 2.5-second ramp, the feet gently took the weight of the robot without triggering explosive ground reaction forces.

## 2. The Left-Leg Splay / Splits ("Positive Feedback Loop")

* **The Symptom:** The left legs (LF, LH) constantly drifted outward until the hip joints hit their physical hard-stops (`1.047` rad), causing the chassis to drop.
* **The Root Cause:** A manual sign inversion (`tau *= -1.0`) was mistakenly applied to the left hips to account for mirroring. However, the URDF (`leg.xacro`) already accounted for the hardware mirroring. Inverting it *again* turned the PD loop into a positive feedback loop: as the leg drifted 1 degree out, the math commanded it to push 2 degrees further out.
* **The Solution:** Removed all manual sign inversions. A pure PD loop `(Target - Actual)` is mathematically self-correcting and will always seek the target automatically when aligned with a properly mirrored URDF.

## 3. Violent High-Frequency Shaking ("Derivative Chattering")

* **The Symptom:** The legs trembled or "buzzed" rapidly while trying to stand, destroying stability. Logs showed torque violently oscillating between positive and negative extremes (+150 Nm to -100 Nm) at 250Hz.
* **The Root Cause:** Gazebo calculates velocity numerically, which results in a highly jagged, noisy velocity signal. A high Damping gain ($K_d = 4.0$) multiplied this microscopic noise into massive, instantaneous torque spikes. (Attempts to fix this with an EMA Low-Pass Filter introduced *Phase Lag*, leading to Integral Windup and rocking).
* **The Solution:** Dropped the Damping gain significantly ($K_d = 0.4$) so it wouldn't overreact to simulation noise. Added a **Velocity Deadband** (`if (abs(vel) < 0.05) vel = 0.0;`) to act as a noise-gate, silencing the microscopic Gazebo jitter completely.

## 4. Resting on Calves ("Steady-State Sag & Torque Starvation")

* **The Symptom:** The robot was stable and perfectly centered, but the back end was sagging, leaving the calf joints resting on the floor.
* **The Root Cause (Part A):** The global torque limit was set to `23.7 Nm`. While correct for the hips and thighs, the Go2 Calves are geared differently and physically require up to `45.43 Nm` of torque. The back end simply didn't have the muscular power to lift the heavy battery.
* **The Root Cause (Part B):** A low, stable stiffness ($K_p = 40.0$) acts like a "spongy" spring. To lift a 12kg robot, a spongy spring must physically compress (sag) to generate enough restoring force.
* **The Solution:** Assigned dynamic, joint-specific clamping limits (`45.43 Nm` for calves). Implemented **Data-Driven Gravity Feedforward ($ff$)** derived directly from steady-state `RAW_TAU` logs. By having the Feedforward directly cancel out the weight of the robot (-2.0 Nm for thighs, +5.0 Nm for calves), the soft $K_p$ springs didn't have to compress at all, resulting in a tall, rigid, and completely motionless stand.

---

Let me know when you are ready to transition this butter-smooth base state over to the Convex MPC solver!