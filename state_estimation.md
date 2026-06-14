# Quadruped Legged State Estimation & Gait Generation Engineering Notes

## 1. Mathematical Framework: 21-State Kinematic-Inertial EKF

The state estimator fuses high-rate inertial measurements (IMU) with discrete kinematic structures derived from joint encoders to track the robot's base state in 3D space.

### State Vector Configuration

The filter tracks a 21-dimensional state vector $\mathbf{x} \in \mathbb{R}^{21}$, structured as:


$$\mathbf{x} = \begin{bmatrix} \mathbf{p}_b & \mathbf{v}_b & \boldsymbol{\theta}_b & \mathbf{p}_{f1} & \mathbf{p}_{f2} & \mathbf{p}_{f3} & \mathbf{p}_{f4} \end{bmatrix}^T$$

Where:

* $\mathbf{p}_b \in \mathbb{R}^3$: Absolute position of the body Center of Mass (CoM) in the world frame.
* $\mathbf{v}_b \in \mathbb{R}^3$: Linear velocity of the body CoM in the world frame.
* $\boldsymbol{\theta}_b \in \mathbb{R}^3$: Spatial orientation error/attitude states in the world frame.
* $\mathbf{p}_{fi} \in \mathbb{R}^3$: Absolute 3D coordinates of Foot $i$ ($i \in \{1, 2, 3, 4\}$) anchored in the world frame.

---

### Process Model (Prediction Step)

The IMU accelerometer $\mathbf{a}_{\text{imu}}$ and gyroscope $\boldsymbol{\omega}_{\text{imu}}$ drive the continuous-time integration step. The state transition updates the body position and velocity over time delta $dt$:


$$\mathbf{p}_b^{(k+1)} = \mathbf{p}_b^{(k)} + \mathbf{v}_b^{(k)}dt + \frac{1}{2}\left(\mathbf{R}(\boldsymbol{\theta}_b^{(k)})\mathbf{a}_{\text{imu}} - \mathbf{g}\right)dt^2$$

$$\mathbf{v}_b^{(k+1)} = \mathbf{v}_b^{(k)} + \left(\mathbf{R}(\boldsymbol{\theta}_b^{(k)})\mathbf{a}_{\text{imu}} - \mathbf{g}\right)dt$$

Where $\mathbf{R}$ represents the rotation matrix converting body-fixed coordinates to the absolute inertial world frame, and $\mathbf{g} = [0, 0, 9.81]^T$.

The covariance prediction updates via the linearized state transition matrix $\mathbf{F} \in \mathbb{R}^{21 \times 21}$ and process noise matrix $\mathbf{Q} \in \mathbb{R}^{21 \times 21}$:


$$\mathbf{P}^{(k+1|k)} = \mathbf{F}\mathbf{P}^{(k|k)}\mathbf{F}^T + \mathbf{Q}$$

The coupling blocks in $\mathbf{F}$ map the velocity dependencies into position, and map the acceleration skew-symmetric cross product $\lfloor\mathbf{a}_{\text{imu}}\times\rfloor$ to capture orientation error drift:


$$\mathbf{F}_{\text{block}(0,3)} = \mathbf{I}_{3 \times 3} \cdot dt, \quad \mathbf{F}_{\text{block}(3,6)} = -\mathbf{R}\lfloor\mathbf{a}_{\text{imu}}\times\rfloor \cdot dt$$

---

### Measurement Model (Correction Step)

When a foot is designated in stance phase, its velocity relative to the absolute world terrain is assumed to be zero ($\dot{\mathbf{p}}_{fi} = \mathbf{0}$). The measurement residual vector $\mathbf{y}_i \in \mathbb{R}^3$ computes the difference between the estimated world foot location and the instantaneous forward kinematics engine projection:


$$\mathbf{y}_i = \mathbf{p}_{fi} - \mathbf{p}_b - \mathbf{R}\cdot\mathbf{r}_{\text{foot\_body}, i}$$

Where $\mathbf{r}_{\text{foot\_body}, i}$ is the Cartesian coordinate vector from the CoM to foot $i$ calculated via forward kinematics. The measurement Jacobian matrix $\mathbf{H}_i \in \mathbb{R}^{3 \times 21}$ maps this relation:


$$\mathbf{H}_i = \begin{bmatrix} -\mathbf{I}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{R}\lfloor\mathbf{r}_{\text{foot\_body}, i}\times\rfloor & \dots & \mathbf{I}_{3 \times 3} & \dots \end{bmatrix}$$

Where the footprint anchor state $\mathbf{I}_{3 \times 3}$ slides dynamically into the respective slot mapping the active foot index ($9 + 3i$).

---

## 2. Debugging Journal & Parameter Tuning Evolution

### Problem 1: Catastrophic Overconfidence Filter Explosion

* **Symptom:** Velocity tracking error drifted continuously past $2.5\text{ m/s}$. The 3-Sigma bound flattened to exactly zero, while the Normalized Innovation Squared (NIS) score shot into the millions.
* **Mathematical Cause:** The process noise parameters inside matrix $\mathbf{Q}$ were severely under-dimensioned (`0.06`). The filter falsely assumed its prediction model was 100% perfect, causing the Kalman Gain matrix to collapse ($\mathbf{K} \to \mathbf{0}$). The filter closed its ears to sensor updates and dead-reckoned blindly on raw IMU integrations.
* **Tuning Execution:** Increased velocity process noise blocks radically to allow open-loop breathing room for structural impact forces:
```cpp
Q_.diagonal().segment<3>(3).setConstant(2.5);  // High velocity process variance allocation
Q_.diagonal().segment<12>(9).setConstant(0.1); // Foot placement freedom expansion

```



### Problem 2: Phase Schedule Inversion & Spatial Tug-of-War

* **Symptom:** Velocity error drifted sharply into negative boundaries ($-2.0\text{ m/s}$), and the NIS profiles spiked rhythmically on every single stride sequence.
* **Mathematical Cause:** The state estimator used an open-loop timing phase clock independent of the locomotion controller. When the robot swung a leg forward at high speed, the EKF clock mistakenly assumed the leg was pinned to the floor in stance ($R_{\text{stance}} = 0.005$). The EKF interpreted the rapid leg movement as the body rocket-propelling backward, corrupting the velocity states.
* **Tuning Execution:** Removed the internal EKF phase clock entirely. Configured a dedicated inter-node communication pipeline utilizing a custom ROS2 broadcasting channel (`/state_estimator/contact_states`) to feed the true, commanded contact states directly from the gait generator.

### Problem 3: Gating Lockout from Suspension Sag

* **Symptom:** The NIS tracker registered a flat line at exactly $0.0$ for extended periods during walking, followed by an immediate multi-thousand point explosion the moment the robot stopped moving.
* **Mathematical Cause:** The hard-coded vertical kinematic validation gate (`r_foot_body.z() > -0.22`) was too strict. In the Gazebo physics engine, the robot's suspension naturally sags downward under load by several centimeters during a trot. Because the sagging leg position breached the gate threshold, the filter classified every active stance foot as an outlier, hitting a `continue;` statement and discarding all leg updates.
* **Tuning Execution:** Relaxed the vertical kinematic gate constraints to accommodate natural joint deflection and compliance under load, and dropped the secondary redundant state-overwrite tracking checks:
```cpp
if (is_stance && (relative_foot_velocity.z() > 0.4 || r_foot_body.z() > -0.12)) { is_stance = false; }

```



### Problem 4: Stale Correlation Matrix Explosion (NaN Propagation)

* **Symptom:** Opening the outlier gate caused the NIS score to spike to $200\text{ Billion}$, and the 3-Sigma plotting line vanished completely from the graph.
* **Mathematical Cause:** When a leg transitions from swing to touchdown, its world position state vector is reset. However, the old cross-covariance rows and columns mapping the correlations between that foot and body velocity were left dirty. This caused matrix $\mathbf{P}$ to lose its positive-definite property. Computing the innovation inverse $\mathbf{S}^{-1}$ resulted in a division-by-zero, propagating `NaN` choices across the matrix stack.
* **Tuning Execution:** Implemented a full cross-covariance wipe routine. On the exact frame a touchdown event occurs, the specific rows and columns matching that leg are zeroed out before injecting the clean baseline tracking variance:
```cpp
P_.block<3, STATE_SIZE>(foot_state_offset, 0).setZero();
P_.block<STATE_SIZE, 3>(0, foot_state_offset).setZero();
P_.block<3, 3>(foot_state_offset, foot_state_offset) = Eigen::Matrix3d::Identity() * 0.1;

```



### Problem 5: Step Length Collapse & Kangaroo Jump Phenomenon

* **Symptom:** The robot hopped erratically and lost its footing the moment command velocities stopped.
* **Mathematical Cause:** Setting `step_len_x = 0.0` inside the gait generator when `has_command` evaluated to false caused a massive position discontinuity. The step length target plummeted to zero in a single millisecond while legs were still mid-stride, forcing the Inverse Kinematics engine to instantly snap the foot positions back to center. The sudden movement generated immense joint torques that launched the robot into the air.
* **Tuning Execution:** Eliminated the hard zero-assignment. Allowed the command low-pass filter (`CMD_ALPHA = 0.08`) to naturally decay velocities to zero, and wrapped a half-cycle horizon boundary guard around the clock to finish active strides gracefully:
```cpp
if (phase_ != 0.0 && phase_ != 0.5) {
    phase_inc = 0.020 * MIN_CADENCE;
    double next_phase = phase_ + phase_inc;
    if (phase_ < 0.5 && next_phase >= 0.5) { phase_ = 0.5; phase_inc = 0.0; }
    else if (phase_ > 0.5 && next_phase >= 1.0) { phase_ = 0.0; phase_inc = 0.0; }
    else { phase_ = std::fmod(next_phase, 1.0); }
}

```



---

## 3. Graphical Diagnostics Cheat Sheet

When reviewing output profiles from the offline tuning script, use this matrix to decode geometric error signatures:

### 3-Sigma Velocity Profiles

* **Smooth, Accelerating Parabolic Drift:** Indicates **Open-Loop Integration**. The filter is ignoring the leg encoders completely and integrating raw IMU accelerometer bias. Check for an outlier gate lockout or topic subscription failure.
* **Noisy, Centered High-Frequency Jitter:** Indicates **Optimal Convergence**. The measurement kinematics are successfully bounding the inertial integration drift. The filter is balanced and closing the loop correctly.
* **Flat Line Fixed at Zero:** Indicates **Numerical Explosion (NaN)**. A calculation error (like a division by zero or an inversion of an unstable matrix) has corrupted the covariance matrix values. Check for cross-covariance reset failures at touchdown.

### NIS (Normalized Innovation Squared) Tracker

* **Parabolic Curve Climbing to Infinity:** Indicates **Gating Lockout**. The estimation state has drifted far from physical reality, causing new measurements to be rejected by the Mahalanobis gate as outliers. Turn off the outlier gate temporarily to allow the filter to snap back to reality.
* **Flat Line Fixed at Exactly 0.0:** Indicates a **State Overwrite Bug**. The residual vector $\mathbf{y}$ is evaluating to zero because the code is continuously forcing the state vector foot positions to match the current kinematics every frame, bypassing the Kalman correction step.
* **Bouncing Signal Between 0.0 and 7.81:** Indicates **Statistical Consistency**. The innovation sequences are operating within the standard 95% confidence intervals for a 3-DOF Chi-Squared distribution.

---

## 4. Architectural Summary: Thought vs. Execution

```
[Twist Command Input] -> [Low-Pass Filter Decay] -> [TrotGait Stride Generator]
                                                           |
                                                (True Contact State Topic)
                                                           v
[Raw IMU Accel/Gyro]   -> [21-State Predict Step] -> [Synchronized EKF Correction Loop]
                                                           |
                                               (Cross-Covariance Zeroed)
                                                           v
                                                [Stable Odometry Output]

```

### Core Implementation Takeaways

1. **Exocentric Clock Rejection:** Never use an independent clock inside an estimation node to guess contact patterns. A stable state estimator must be directly synchronized with the locomotion layer's step states.
2. **Covariance Isolation:** Resetting state estimations requires cleaning up the associated correlation parameters. When anchoring a state variable back to a physical reference, the cross-covariance values must be zeroed out to maintain matrix stability.
3. **Kinematic Continuity Rules:** Never allow geometric targets inside a locomotion control loop to jump abruptly. All state changes, velocity shutdowns, and touchdown terminations must decay smoothly to preserve joint torque stability.