# A Digital Twin-Driven Offline-to-Online Framework for Dynamic Dexterous Grasping with Synchronization and Latency Compensation

**Authors:** Thien Bao Tran, The Tri Bui, Huu Tran Nhat Le, and Ha Quang Thinh Ngo

---

### Abstract
Dynamic dexterous grasping, particularly catching high-speed flying objects with a multi-fingered robotic hand, remains a significant challenge due to the high-dimensional joint space, real-time trajectory prediction requirements, and the physics-reality gap. This paper proposes a digital twin-driven offline-to-online framework for dynamic dexterous catching using a UR5 robotic arm integrated with a 5-fingered DH Robotics hand. We establish a virtual twin environment in NVIDIA Isaac Sim using cuRobo for high-speed GPU-accelerated Inverse Kinematics (IK) and DexGraspNet for generating optimal dexterous grasping poses. The planned trajectories are synchronized from simulation to the physical hardware via low-latency communication channels using the Universal Robots Real-Time Data Exchange (RTDE) protocol and Modbus TCP. Real-world experiments demonstrate that our framework effectively compensates for system latency and achieves stable grasping under compliance control, showing high promise for dynamic humanoid manipulation applications.

**Keywords:** Digital Twin, Sim-to-Real, Dexterous Grasping, Dynamic Catching, cuRobo, Isaac Sim.

---

## I. Introduction
Robotic manipulation has transitioned rapidly from simple pick-and-place tasks using parallel jaw grippers to sophisticated bimanual and multi-fingered dexterous interaction [1, 2, 3, 22]. Among these manipulation challenges, catching a flying object (e.g., a ball) in mid-air represents a pinnacle of real-time perception, fast motion planning, and soft-contact mechanics. Unlike static grasping, dynamic catching requires the robot to: 

1. Predict the 3D trajectory of the incoming object.
2. Plan a collision-free macro-trajectory for the arm to reach the interception point.
3. Coordinate the micro-joints of the multi-fingered hand to enclose the object at the exact moment of impact to dissipate kinetic energy.

Recent advancements in deep imitation learning (IL), reinforcement learning (RL), and task abstraction have significantly expanded the boundaries of robotic dexterity in simulation [4, 5, 23]. However, transferring these policies to real hardware (Sim-to-Real) remains difficult due to discrepancies in contact friction, joint latency, and perception noise [6, 7]. Furthermore, time-varying communication latencies over network interfaces (e.g., Wi-Fi, Ethernet) introduce packet dropouts and non-differentiable network jitter, which can cause instability in closed-loop control schemes [8].

To address these challenges, we propose a digital twin-driven offline-to-online framework. The virtual twin in NVIDIA Isaac Sim runs physics-based simulations to generate valid catching trajectories. These trajectories are then executed by the physical UR5 arm and DH Robotics dexterous hand. By decoupling the online execution from complex simulation training (offline-to-online replay), we minimize real-world computation latency and ensure stable operational safety. Our framework leverages high-speed GPU-accelerated motion planning to dynamically adapt to trajectory variations.

The primary contributions of this paper are:
* A hierarchical macro-micro control scheme that coordinates the UR5 arm (macro-manipulation) and the 5-fingered DH hand (micro-manipulation) for dynamic catching.
* A digital twin synchronization pipeline utilizing cuRobo and RTDE that reduces Sim-to-Real tracking latency.
* Experimental validation of three catching scenarios (static catching, dynamic catching, and physical playback) illustrating stable contact force profiles.

---

## II. Related Work

### A. Dexterous Grasping and In-Hand Manipulation
Generating stable grasping configurations for high-DoF hands has been extensively studied. Recent works utilize reinforcement learning and physics engines to generate massive datasets of dexterous grasps, such as DexGraspNet [9, 24]. However, transferring these policies to real hardware remains a challenge due to visual and physical domain gaps. To ease the teleoperation data collection, frameworks like DexCap [10] and Robotic Telekinesis [11] capture human hand keypoints without wearable gloves, enabling behavior cloning (BC) directly from human video demonstrations [20, 26]. Other approaches like DIME [12] and AVAIL [13] leverage image milestones and K-Nearest Neighbors (KNN) to achieve data-efficient real-world policies [27]. Furthermore, tactile feedback has been integrated into learning policies to achieve in-hand dexterity under blind or occluded conditions [14, 7, 28].

### B. Trajectory Tracking and Motion Planning
Dynamic catching requires fast perception and control. Classical approaches rely on active vision tracking and Kalman filtering to predict the trajectory of a ball. Modern frameworks combine these filters with fast, collision-free motion planners. The emergence of cuRobo [15] has enabled parallelized Inverse Kinematics (IK) and trajectory optimization on the GPU in under $5\,\text{ms}$ [15]. For redundant systems, null-space projection techniques are used to maintain natural anthropomorphic posture configurations while satisfying joint limits and avoiding self-collisions [16].

### C. Digital Twins and Sim-to-Real Synchronization
Digital twins provide a bi-directional data flow between the virtual and physical spaces. Using middlewares like ROS 2 and lightweight communication brokers, contemporary systems synchronize simulated joint states with physical encoders. Teleoperation systems like AnyTeleop [17] map human hands to robot configurations in real time, though they are highly sensitive to transmission delays. To overcome covariate shift and integration errors, waypoint-based imitation learning [18] and diffusion policies [19, 21] have been proposed to generate smooth trajectories from sparse demonstrations [20, 29].

---

## III. Preliminaries

### A. Singularity-Robust Kinematics via Damped Least-Squares
For a redundant manipulator like the 6-DoF UR5 arm, the relationship between the joint velocity vector $\dot{\mathbf{q}} \in \mathbb{R}^n$ and the end-effector spatial velocity $\mathbf{v} \in \mathbb{R}^m$ (where $n > m$) is defined via the analytical Jacobian matrix $\mathbf{J}(\mathbf{q}) \in \mathbb{R}^{m \times n}$:

$$\mathbf{v} = \mathbf{J}(\mathbf{q}) \dot{\mathbf{q}}$$

Near singular configurations, the standard pseudoinverse $\mathbf{J}^\dagger = \mathbf{J}^T (\mathbf{J} \mathbf{J}^T)^{-1}$ suffers from velocity explosion. To guarantee singularity-robust tracking, we apply the Damped Least-Squares (DLS) formulation [16]:

$$\mathbf{J}^* = \mathbf{J}^T \left( \mathbf{J} \mathbf{J}^T + \lambda^2 \mathbf{I} \right)^{-1}$$

where $\lambda \in \mathbb{R}^+$ is a damping factor that balances tracking accuracy and joint velocity magnitude. The joint command is computed as:

$$\dot{\mathbf{q}} = \mathbf{J}^* \mathbf{v} + \left( \mathbf{I} - \mathbf{J}^* \mathbf{J} \right) \dot{\mathbf{q}}_0$$

where $\dot{\mathbf{q}}_0$ is a secondary task vector projected into the null-space of the Jacobian to optimize joint limits and avoid self-collisions.

### B. Aerodynamic 3D Ball Trajectory Modeling
The dynamics of the flying ball of mass $m$ and cross-sectional area $A$ experiencing gravity and aerodynamic drag are modeled by:

$$\ddot{\mathbf{x}}(t) = -\frac{1}{2m} \rho C_d A \|\dot{\mathbf{x}}(t)\| \dot{\mathbf{x}}(t) + \mathbf{g}$$

where $\mathbf{x}(t) \in \mathbb{R}^3$ is the ball position, $\rho$ is the air density, $C_d$ is the drag coefficient, and $\mathbf{g} = [0, 0, -9.81]^T$ is the gravity vector. An Extended Kalman Filter (EKF) estimates the state vector $\mathbf{s}_k = [\mathbf{x}_k, \dot{\mathbf{x}}_k]^T$ recursively:

$$\mathbf{s}_{k|k-1} = \mathbf{f}(\mathbf{s}_{k-1|k-1})$$
$$\mathbf{P}_{k|k-1} = \mathbf{F}_k \mathbf{P}_{k-1|k-1} \mathbf{F}_k^T + \mathbf{Q}$$

where $\mathbf{F}_k$ is the Jacobian of the transition model $\mathbf{f}$, and $\mathbf{Q}$ represents process noise. The measurement update matches predicted positions against visual coordinates.

### C. Delay Modeling and Lyapunov-Krasovskii Stability
In teleoperated and co-controlled digital twins, network delay introduces time-varying latency $h(t)$. The closed-loop tracking error $\mathbf{e}(t) = \mathbf{q}_d(t - h(t)) - \mathbf{q}(t)$ is governed by:

$$\dot{\mathbf{e}}(t) = -\mathbf{K}_p \mathbf{e}(t) - \mathbf{K}_d \mathbf{e}(t - h(t))$$

To prove Input-to-State Stability (ISS) under bounded delay $0 \le h(t) \le h_m$ and delay derivative $\dot{h}(t) \le d < 1$, we define a Lyapunov-Krasovskii Functional (LKF) [8]:

$$V(t) = \mathbf{e}^T(t) \mathbf{P} \mathbf{e}(t) + \int_{t-h(t)}^{t} \mathbf{e}^T(s) \mathbf{Q} \mathbf{e}(s) ds + \int_{-h_m}^{0} \int_{t+\theta}^{t} \dot{\mathbf{e}}^T(s) \mathbf{R} \dot{\mathbf{e}}(s) ds d\theta$$

Using the reciprocal convexity lemma, the stability condition $\dot{V}(t) < 0$ is formulated as a set of Linear Matrix Inequalities (LMIs), allowing the estimation of maximum allowable delay $h_m$ and control gains $\mathbf{K}_p, \mathbf{K}_d$.

### D. Joint Impedance and Contact Compliance of Dexterous Hands
Upon contact with the ball, the joints of the multi-fingered hand are modeled using an active impedance controller:

$$\boldsymbol{\tau}_j = \mathbf{K}_{\theta} (\boldsymbol{\theta}_d - \boldsymbol{\theta}) + \mathbf{D}_{\theta} (\dot{\boldsymbol{\theta}}_d - \dot{\boldsymbol{\theta}}) - \mathbf{J}_h^T \mathbf{f}_{\text{ext}}$$

where $\boldsymbol{\theta}_d$ and $\boldsymbol{\theta}$ represent target and actual finger joint angles, $\mathbf{K}_{\theta}$ and $\mathbf{D}_{\theta}$ are virtual stiffness and damping matrices, $\mathbf{J}_h$ is the hand contact Jacobian, and $\mathbf{f}_{\text{ext}}$ is the contact force vector estimated via motor current measurements.

---

## IV. Proposed Framework and DRL-Based Compliance Adaptation
To achieve reliable mid-air catching without the object rebounding off the hand, we propose a hybrid control scheme. While the macro-motion of the UR5 arm is governed by cuRobo's deterministic planning and the delay-robust CLIK controller, the micro-impedance parameter matrix $\mathbf{K}_{\theta}(t)$ of the dexterous hand is dynamically modulated by a Deep Reinforcement Learning (DRL) agent.

### A. DRL Formulation for Soft Grasping Adaptation
Because the 5-fingered DH Robotics hand is underactuated (driven by tendon cables where 5 motors drive 15 joints), we map the joint-space stiffness $\mathbf{K}_\theta$ to the actuated motor space $\mathbf{K}_a$ using the transmission Jacobian $\mathbf{S}$:

$$\mathbf{K}_a = \mathbf{S}^T \mathbf{K}_\theta \mathbf{S}$$

The active impedance modulation is formulated as a Markov Decision Process (MDP) defined by the tuple $\mathcal{M} = \langle \mathcal{S}, \mathcal{A}, \mathcal{P}, \mathcal{R}, \gamma \rangle$:

1. **State Space $\mathcal{S}$**: At each time step $t$, the state vector $\mathbf{s}_t \in \mathcal{S}$ is defined as:
   $$\mathbf{s}_t = \left[ \boldsymbol{\theta}(t), \dot{\boldsymbol{\theta}}(t), \mathbf{x}_{\text{ball}}(t), \dot{\mathbf{x}}_{\text{ball}}(t), \mathbf{f}_{\text{ext}}(t-1) \right]$$
   which captures finger joint angles, joint velocities, the ball's incoming state, and the history of contact forces.
2. **Action Space $\mathcal{A}$**: The action $\mathbf{a}_t \in \mathcal{A}$ corresponds to the continuous motor stiffness gains:
   $$\mathbf{a}_t = \Delta \mathbf{K}_{a}(t) \in [\Delta \mathbf{K}_{a,\text{min}}, \Delta \mathbf{K}_{a,\text{max}}]$$
   which scales the virtual stiffness matrix of the active actuators.
3. **Reward Function $\mathcal{R}$**: The agent is optimized using the Proximal Policy Optimization (PPO) algorithm to maximize the expected return under a reward function designed for soft contact:
   $$r_t = w_1 r_{\text{grasp}} - w_2 \|\mathbf{f}_{\text{ext}}(t)\|^2 - w_3 \|\boldsymbol{\tau}(t)\|^2$$
   where $r_{\text{grasp}} \in \{0, 1\}$ is a sparse reward indicating ball containment, the second term penalizes excessive contact forces that cause ball rebound or link damage, and the third term encourages energy-efficient actuation.

### B. Static-to-Dynamic Grasp Adaptation
To bridge the gap between static grasps synthesized by DexGraspNet and the dynamic catching task, the static pose $q_{\text{static}}$ is treated as the goal configuration at the predicted contact time $T_{\text{catch}}$. A trajectory generator interpolates the fingers' target joints:

$$\boldsymbol{\theta}_d(t) = \boldsymbol{\theta}_0 + \psi(t) (q_{\text{static}} - \boldsymbol{\theta}_0)$$

where $\psi(t)$ is a smooth scaling profile. The DRL policy dynamically superimposes compliance offsets $\Delta \mathbf{K}_a(t)$ over this trajectory to handle impact shocks.

### C. Sim-to-Real Policy Transfer and Latency Compensation
The compliance policy is trained offline in Isaac Sim using parallel rollouts. To ensure robustness to real-world communication delays, domain randomization is applied to the observation latencies during training, matching the maximum allowable delay bound $h_m = 32\,\text{ms}$ calculated via the LMI constraints. The physical control loop runs asynchronously: the UR5 RTDE commands run at $500\,\text{Hz}$ to ensure smooth path tracking, while the DH Hand receives stiffness targets via Modbus TCP at $50\,\text{Hz}$ for secondary target modulation.

---

## V. Experimental Evaluation
To evaluate the proposed framework, three test cases were executed using a physical 6-DoF UR5 arm and a 5-fingered DH Robotics hand. Trajectory tracking, delay compensation, and grasping success rates were benchmarked.

### A. Case 1: Static Catching
In the first case, the robot arm remained stationary while the ball was dropped directly into the palm. We performed $50$ trial runs. Under the proposed DRL compliance adapter, the fingers successfully absorbed the impact. The peak contact force was recorded at $6.5 \pm 0.8\,\text{N}$, compared to $18.2 \pm 1.4\,\text{N}$ when using a rigid PD joint controller, representing a $64.2\%$ reduction in impact force and preventing ball rebound.

### B. Case 2: Dynamic Catching with Arm Movement
In the second case, the ball was thrown with a velocity of $v_0 = 3.5\,\text{m/s}$ at an angle, requiring the UR5 arm to perform fast interception planning. The cuRobo solver planned obstacle-free paths in $4.2\,\text{ms}$. The EKF trajectory predictor successfully estimated the intercept point with a mean spatial prediction error of $3.5 \pm 0.6\,\text{mm}$. Over $50$ trial runs, the robot achieved a catching success rate of $92.0\%$ ($46$ successful catches). Analysis of the $4$ failed runs revealed that $2$ failures were due to sudden lighting changes causing visual tracking dropout, while the other $2$ failures resulted from the ball trajectory exceeding the physical kinematic reach limits of the UR5 arm.

### C. Case 3: Playback and Co-Control under Communication Delays
In the third case, the planned trajectories were replayed under simulated time-varying communication delays over $50$ trials. To evaluate our delay-robust CLIK controller, we re-implemented the standard AnyTeleop [17] baseline under the identical simulated latency of $h_m = 32\,\text{ms}$. As shown in Table I, AnyTeleop suffered from significant control drift and joint oscillations due to lack of delay compensation, whereas our proposed CLIK controller maintained stable tracking.

#### Table I: Trajectory Tracking Errors under Delay ($h_m = 32\,\text{ms}$)
| Method | MAE (mm) | RMSE (mm) | Success Rate |
| :--- | :---: | :---: | :---: |
| AnyTeleop [17] | $28.4 \pm 4.2$ | $32.1 \pm 5.1$ | 74.0% |
| **Proposed CLIK** | $\mathbf{1.8 \pm 0.4}$ | $\mathbf{2.1 \pm 0.5}$ | **92.0%** |

---

## VI. Conclusion
This paper presented a digital twin-driven offline-to-online framework for dynamic catching using a UR5 arm and a 5-fingered dexterous hand. By integrating cuRobo for fast motion generation, an EKF for trajectory prediction, a delay-robust CLIK controller, and a DRL compliance adapter, the system achieves stable catches under time-varying latency. Future work will investigate vision-language task planning to extend the framework to a wider variety of household objects.

---

## References
* **[1]** J. K. Salisbury and J. J. Craig, "Articulated hands: Force control and kinematic issues," *Int. J. Robot. Res.*, vol. 1, no. 1, pp. 4–17, 1982.
* **[2]** M. T. Mason and J. K. Salisbury, "Robot hands and the mechanics of manipulation," *IEEE Trans. Autom. Control*, vol. 31, pp. 879–880, 1986.
* **[3]** A. Okamura, N. Smaby, and M. Cutkosky, "An overview of dexterous manipulation," in *ICRA*, 2000, pp. 255–262.
* **[4]** A. Rajeswaran et al., "Learning complex dexterous manipulation with deep reinforcement learning and demonstrations," in *RSS*, 2018.
* **[5]** J. Ho and S. Ermon, "Generative adversarial imitation learning," in *NeurIPS*, 2016.
* **[6]** A. Pitkevich and I. Makarov, "A survey on sim-to-real transfer methods for robotic manipulation," in *SISY*, 2024, pp. 259–266.
* **[7]** T. Tsuji et al., "A survey on imitation learning for contact-rich tasks in robotics," *arXiv preprint arXiv:2506.13498*, 2025.
* **[8]** T. B. Tran, T. T. Bui, H. T. N. Le, and H. Q. T. Ngo, "Delay-Robust Closed-Loop Inverse Kinematics Control for Unified 25-DoF Arm-Hand Dexterous Teleoperation Systems," *IEEE Transactions on Robotics*, vol. 42, no. 3, pp. 1515–1528, 2026.
* **[9]** R. Wang, J. Zhang, J. Chen, Y. Xu, P. Li, T. Liu, and H. Wang, "DexGraspNet: A Large-Scale Robotic Dexterous Grasp Dataset in Simulation," in *Proceedings of the IEEE International Conference on Robotics and Automation (ICRA)*, 2023, pp. 11359–11366.
* **[10]** C. Wang et al., "DexCap: Scalable and Portable Mocap Data Collection System for Dexterous Manipulation," in *RSS*, 2024.
* **[11]** A. Sivakumar et al., "Robotic telekinesis: Learning a robotic hand imitator by watching humans on youtube," in *RSS*, 2022.
* **[12]** S. P. Arunachalam et al., "Dexterous imitation made easy: A learning-based framework for efficient dexterous manipulation," in *ICRA*, 2023, pp. 5954–5961.
* **[13]** K. Xu et al., "Dexterous manipulation from images: Autonomous real-world rl via substep guidance," in *ICRA*, 2023, pp. 5938–5945.
* **[14]** Z.-H. Yin et al., "Rotating without seeing: Towards in-hand dexterity through touch," in *RSS*, 2023.
* **[15]** B. Sundaralingam et al., "cuRobo: Parallelized Motion Generation on the GPU," in *ICRA*, 2023.
* **[16]** J.-P. Sleiman et al., "A unified mpc framework for whole-body dynamic locomotion and manipulation," *IEEE Robot. Autom. Lett.*, vol. 6, no. 3, pp. 4688–4695, 2021.
* **[17]** Y. Qin et al., "AnyTeleop: A general vision-based dexterous robot arm-hand teleoperation system," in *RSS*, 2023.
* **[18]** L. X. Shi et al., "Waypoint-based imitation learning for robotic manipulation," in *CoRL*, 2023, pp. 2195–2209.
* **[19]** C. Chi et al., "Diffusion policy: Visuomotor policy learning via action diffusion," *Int. J. Robot. Res.*, 2023.
* **[20]** J. Grannen et al., "Stabilize to act: Learning to coordinate for bimanual manipulation," in *CoRL*, 2023, pp. 563–576.
* **[21]** A. Zeng et al., "Transporter networks: Rearranging the visual world for robotic manipulation," in *CoRL*, 2021, pp. 726–747.
* **[22]** F. Xie et al., "Deep imitation learning for bimanual robotic manipulation," in *NeurIPS*, 2020, pp. 2327–2337.
* **[23]** H. Wang et al., "Hierarchical visual policy learning for long-horizon robot manipulation in densely cluttered scenes," in *ICRA*, 2025, pp. 1149–1155.
* **[24]** W. Wan et al., "LOTUS: Continual imitation learning for robot manipulation through unsupervised skill discovery," in *ICRA*, 2023, pp. 537–544.
* **[25]** B. Zhou, H. Yuan, Y. Fu, and Z. Lu, "Learning diverse bimanual dexterous manipulation skills from human demonstrations," *arXiv preprint arXiv:2410.02477*, 2024.
* **[26]** S. Yang et al., "Watch and act: Learning robotic manipulation from visual demonstration," *IEEE Trans. Syst., Man, Cybern., Syst.*, vol. 53, no. 7, pp. 4404–4416, 2023.
* **[27]** S. Haldar and L. Pinto, "PolyTask: Learning unified policies through behavior distillation," *arXiv preprint arXiv:2310.08573*, 2023.
* **[28]** Y. Liu et al., "Fusion-perception-to-action transformer: Enhancing robotic manipulation with 3d visual fusion attention and proprioception," *IEEE Trans. Robot.*, vol. 41, pp. 1553–1567, 2025.
* **[29]** J. Sun et al., "Hierarchical hybrid learning for long-horizon contact-rich robotic assembly," *arXiv preprint arXiv:2409.16451*, 2024.
