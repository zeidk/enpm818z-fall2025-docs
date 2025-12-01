==================================================================
Final Project
==================================================================

:Title: Trajectory Planning in Frenet Coordinates

:Course: ENPM818Z — On-Road Automated Vehicles
:Topic: L6 — Trajectory Planning
:Assigned: December 1, 2025
:Due: December 12, 2025
:Total Points: 60 pts
:Language: Python (CARLA)

.. admonition:: Resources
   :class: resources

   - 🔗 `Starter package <https://github.com/zeidk/enpm818Z-fall-2025/tree/main/rwa3_final_starter>`_
   
   - 📖 Lecture Materials: L6 (Trajectory Planning)
   
   - 📖 CARLA Documentation: https://carla.readthedocs.io/

.. admonition:: Changelog

   **Version 1.0.0** (2025-11-30)

   - First version



.. note::

   **Project Structure:**
   
   This final project focuses on **Trajectory Planning** using Frenet coordinates and polynomial trajectories (Part 2 of the Planning Stack). This builds upon **Assignment 3** (Behavioral Planning with Behavior Trees). Together, these implement a complete hierarchical planning system for autonomous driving.
   
   **Prerequisite:** You must have completed Assignment 3 (Behavioral Planner) before starting this project.

---------------------------------------------------------
1. Objective
---------------------------------------------------------

Implement a **trajectory planner** that generates smooth, collision-free trajectories using the Frenet optimal trajectory method. Your implementation will integrate with the behavioral planner from Assignment 3, along with provided perception and control modules, to create a complete autonomous driving system.

**Learning Objectives:**

- Implement Frenet coordinate transformation between Cartesian and road-relative frames
- Generate polynomial trajectories that satisfy boundary conditions
- Design and tune cost functions for trajectory evaluation
- Implement feasibility checking for vehicle dynamic constraints
- Implement collision checking with dynamic obstacles
- Test and validate trajectory planning in simulation

---------------------------------------------------------
2. Background
---------------------------------------------------------

In the lecture, we discussed how the **trajectory planner** sits between behavioral planning and control. While the behavioral planner decides *what* to do (e.g., change lanes), the trajectory planner determines *how* to do it—generating a smooth path through space and time.

**Frenet Coordinates:**

The Frenet frame provides a road-relative coordinate system that simplifies trajectory planning:

- **s (arc length):** Distance traveled along the road centerline
- **d (lateral offset):** Perpendicular distance from the centerline (positive = left)

This representation decouples longitudinal and lateral motion, making it easier to plan lane changes and speed adjustments independently.

**Polynomial Trajectories:**

We use polynomial functions to represent motion over time:

- **Quintic polynomials** (5th order) for lateral motion: 6 coefficients match 6 boundary conditions (initial and final position, velocity, acceleration)
- **Quartic polynomials** (4th order) for longitudinal motion: 5 coefficients match 5 boundary conditions (initial state + final velocity/acceleration)

**Optimal Trajectory Selection:**

The planner generates multiple candidate trajectories by sampling different end states (target lateral position, velocity, duration). Each candidate is evaluated for feasibility (dynamic constraints) and cost (smoothness, efficiency, safety). The lowest-cost feasible trajectory is selected.

**Why This Matters:**

This approach, known as the "Frenet Optimal Trajectory" method, is widely used in autonomous driving systems. It provides a principled way to generate smooth, safe trajectories that respect vehicle dynamics and avoid obstacles.

---------------------------------------------------------
3. System Architecture
---------------------------------------------------------

The trajectory planner receives commands from the behavioral planner and outputs trajectories to the controller.

Data Flow
~~~~~~~~~

.. list-table::
   :widths: 25 75
   :header-rows: 1
   :class: compact-table

   * - **Module**
     - **Description**
   * - **Perception (Provided)**
     - Provides ego state, obstacles, lane info, and reference path
   * - **Behavioral Planner (From RWA3)**
     - Outputs ``BehavioralCommand`` with maneuver type and targets
   * - **Trajectory Planner (You Implement)**
     - Generates smooth, collision-free trajectory in Cartesian coordinates
   * - **Controller (Provided)**
     - Tracks the trajectory using Stanley (lateral) and PID (longitudinal) control

What You Receive vs. Implement
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 40 15 15
   :header-rows: 1
   :class: compact-table

   * - **Module**
     - **Provided**
     - **You Implement**
   * - CARLA Interface (``carla_interface.py``)
     - ✓
     - 
   * - Perception Module (``perception.py``)
     - ✓
     - 
   * - Behavior Tree Framework (``bt_framework.py``)
     - ✓ 
     - 
   * - Behavior Tree Nodes (``bt_nodes.py``)
     - 
     - ✓ (Assignment 3)
   * - Behavioral Planner (``behavioral_planner.py``)
     - 
     - ✓ (Assignment 3)
   * - Frenet Transform (``frenet_transform.py``)
     - 
     - ✓ (Assignment 3)
   * - Polynomial Trajectory (``polynomial_trajectory.py``)
     - 
     - ✓ (This Assignment)
   * - Trajectory Planner (``trajectory_planner.py``)
     - 
     - ✓ (This Assignment)
   * - Controller (``controller.py``)
     - ✓
     - 
   * - Visualization (``visualization.py``)
     - ✓
     - 


Key Interfaces
~~~~~~~~~~~~~~

**Behavioral Planner → Trajectory Planner:**

.. code-block:: python

   BehavioralCommand:
       maneuver: str       # 'lane_keep' | 'follow' | 'lane_change_left' | 'lane_change_right' | 'stop'
       target_lane: int    # Target lane ID
       target_speed: float # Target speed in m/s

**Trajectory Planner → Controller:**

.. code-block:: python

   Trajectory:
       points: List[TrajectoryPoint]  # Sequence of waypoints
       cost: float                     # Total cost
       feasible: bool                  # Passed constraint checks
       collision_free: bool            # Passed collision checks
   
   TrajectoryPoint:
       t: float      # Time (seconds)
       x: float      # Global X position (meters)
       y: float      # Global Y position (meters)
       theta: float  # Heading (radians)
       v: float      # Speed (m/s)
       kappa: float  # Curvature (1/m)
       a: float      # Acceleration (m/s²)

---------------------------------------------------------
4. Configuration Parameters
---------------------------------------------------------

The trajectory planner uses parameters defined in ``config/planner_config.yaml``. Understanding these parameters is essential for correct implementation.

Vehicle Constraint Parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 30 15 55
   :header-rows: 1
   :class: compact-table

   * - **Parameter**
     - **Default**
     - **Description**
   * - ``max_speed``
     - 30.0 m/s
     - Maximum allowable vehicle speed (~108 km/h). Trajectories exceeding this are infeasible.
   * - ``max_accel``
     - 3.0 m/s²
     - Maximum comfortable acceleration. Represents typical passenger comfort limits.
   * - ``max_decel``
     - -6.0 m/s²
     - Maximum comfortable deceleration (negative value). Emergency braking can reach -8 m/s².
   * - ``max_curvature``
     - 0.2 m⁻¹
     - Maximum path curvature, corresponding to minimum turning radius of 5 meters.
   * - ``max_lateral_accel``
     - 3.0 m/s²
     - Maximum lateral acceleration (v² × κ). Exceeding this causes passenger discomfort.
   * - ``max_jerk``
     - 2.0 m/s³
     - Maximum rate of change of acceleration. Important for ride comfort.

Planning Parameters
~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 30 15 55
   :header-rows: 1
   :class: compact-table

   * - **Parameter**
     - **Default**
     - **Description**
   * - ``planning_horizon``
     - 5.0 s
     - How far into the future to plan trajectories.
   * - ``dt``
     - 0.1 s
     - Time step for trajectory discretization (10 Hz).
   * - ``num_d_samples``
     - 5
     - Number of lateral position samples around target.
   * - ``num_v_samples``
     - 5
     - Number of velocity samples around target speed.
   * - ``num_t_samples``
     - 5
     - Number of duration samples for trajectory completion time.
   * - ``d_sample_range``
     - 0.5 m
     - Range (±) for lateral position sampling around target.
   * - ``v_sample_range``
     - 2.0 m/s
     - Range (±) for velocity sampling around target speed.
   * - ``t_sample_min``
     - 3.0 s
     - Minimum trajectory duration for sampling.
   * - ``t_sample_max``
     - 6.0 s
     - Maximum trajectory duration for sampling.

Cost Function Weights
~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 25 15 60
   :header-rows: 1
   :class: compact-table

   * - **Weight**
     - **Default**
     - **Description**
   * - ``jerk``
     - 0.1
     - Penalizes jerky motion (integral of squared jerk). Lower weight allows more aggressive maneuvers.
   * - ``lateral_deviation``
     - 1.0
     - Penalizes deviation from target lateral position. Higher weight keeps vehicle closer to lane center.
   * - ``speed_deviation``
     - 1.0
     - Penalizes deviation from target speed. Higher weight prioritizes speed matching.
   * - ``time``
     - 0.5
     - Penalizes longer trajectories. Higher weight prefers faster maneuvers.
   * - ``obstacle_proximity``
     - 10.0
     - Penalizes trajectories passing close to obstacles. Critical for safety.

Vehicle Dimensions
~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 25 15 60
   :header-rows: 1
   :class: compact-table

   * - **Parameter**
     - **Default**
     - **Description**
   * - ``vehicle_length``
     - 4.5 m
     - Length of ego vehicle for collision checking.
   * - ``vehicle_width``
     - 2.0 m
     - Width of ego vehicle for collision checking.
   * - ``safety_margin``
     - 1.0 m
     - Additional buffer added to vehicle dimensions during collision checks.
   * - ``lane_width``
     - 3.5 m
     - Standard lane width for computing target lateral positions.

**Example Configuration:**

.. code-block:: yaml

   trajectory_planner:
     # Vehicle constraints
     max_speed: 30.0
     max_accel: 3.0
     max_decel: -6.0
     max_curvature: 0.2
     max_lateral_accel: 3.0
     
     # Planning parameters
     planning_horizon: 5.0
     dt: 0.1
     num_d_samples: 5
     num_v_samples: 5
     num_t_samples: 5
     
     # Cost weights
     cost_weights:
       jerk: 0.1
       lateral_deviation: 1.0
       speed_deviation: 1.0
       time: 0.5
       obstacle_proximity: 10.0

---------------------------------------------------------
5. Assignment Tasks
---------------------------------------------------------

You will implement three modules: **Frenet Transform**, **Polynomial Trajectory**, and **Trajectory Planner**.

Task 1: Frenet Coordinate Transformation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**File:** ``frenet_transform.py``

Implement functions to convert between Cartesian (x, y) and Frenet (s, d) coordinates.


.. prf:algorithm:: cartesian_to_frenet
   :label: cartesian-to-frenet

   **Inputs:**
   
   - :math:`x, y` (global position in meters)
   - :math:`\theta` (heading in radians)
   - :math:`v` (speed in m/s)
   - :math:`path` (list of (x, y) waypoints defining reference line)
   
   **Output:** FrenetState with s, d, s_dot, d_dot

   1. Compute path properties:
   
      a. :math:`s_{values}, headings, curvatures, path\_array \gets \text{compute_reference_path_properties}(path)`
   
   2. Find closest point on path:
   
      a. :math:`idx, s, dist \gets \text{find_closest_point}(x, y, path\_array, s_{values})`
   
   3. Get path properties at closest point:
   
      a. :math:`path_x \gets path\_array[idx, 0]`
      
      b. :math:`path_y \gets path\_array[idx, 1]`
      
      c. :math:`\psi \gets headings[idx]` (path heading)
   
   4. Compute lateral offset d:
   
      a. :math:`dx \gets x - path_x`
      
      b. :math:`dy \gets y - path_y`
      
      c. :math:`n_x \gets -\sin(\psi)` (normal vector x-component)
      
      d. :math:`n_y \gets \cos(\psi)` (normal vector y-component)
      
      e. :math:`d \gets dx \cdot n_x + dy \cdot n_y`
   
   5. Compute velocity components:
   
      a. :math:`\Delta\theta \gets \theta - \psi`
      
      b. :math:`\Delta\theta \gets \text{normalize_angle}(\Delta\theta)` (to [-π, π])
      
      c. :math:`\dot{s} \gets v \cdot \cos(\Delta\theta)`
      
      d. :math:`\dot{d} \gets v \cdot \sin(\Delta\theta)`
   
   6. **return** :math:`\text{FrenetState}(s, d, \dot{s}, \dot{d})`


.. prf:algorithm:: frenet_to_cartesian
   :label: frenet-to-cartesian

   **Inputs:**
   
   - :math:`s` (arc length along path in meters)
   - :math:`d` (lateral offset in meters, positive = left)
   - :math:`\dot{s}` (longitudinal velocity in m/s)
   - :math:`\dot{d}` (lateral velocity in m/s)
   - :math:`path` (list of (x, y) waypoints)
   
   **Output:** CartesianState with x, y, theta, v

   1. Compute path properties:
   
      a. :math:`s_{values}, headings, curvatures, path\_array \gets \text{compute_reference_path_properties}(path)`
   
   2. Find point on path at arc length s (interpolate):
   
      a. :math:`idx \gets \text{searchsorted}(s_{values}, s) - 1`
      
      b. :math:`idx \gets \text{clamp}(idx, 0, len(s_{values}) - 2)`
      
      c. :math:`t \gets (s - s_{values}[idx]) / (s_{values}[idx+1] - s_{values}[idx])`
      
      d. :math:`path_x \gets path\_array[idx, 0] + t \cdot (path\_array[idx+1, 0] - path\_array[idx, 0])`
      
      e. :math:`path_y \gets path\_array[idx, 1] + t \cdot (path\_array[idx+1, 1] - path\_array[idx, 1])`
      
      f. :math:`\psi \gets headings[idx] + t \cdot (headings[idx+1] - headings[idx])`
   
   3. Compute Cartesian position:
   
      a. :math:`x \gets path_x + d \cdot (-\sin(\psi))`
      
      b. :math:`y \gets path_y + d \cdot \cos(\psi)`
   
   4. Compute heading:
   
      a. **if** :math:`|\dot{s}| > 0.01` **then** :math:`\theta \gets \psi + \arctan2(\dot{d}, \dot{s})`
      
      b. **else** :math:`\theta \gets \psi`
   
   5. Compute speed:
   
      a. :math:`v \gets \sqrt{\dot{s}^2 + \dot{d}^2}`
   
   6. **return** :math:`\text{CartesianState}(x, y, \theta, v)`


Task 2: Polynomial Trajectory Generation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**File:** ``polynomial_trajectory.py``

Implement functions to compute polynomial coefficients and evaluate trajectories.


.. prf:algorithm:: quintic_coefficients
   :label: quintic-coefficients

   **Inputs:**
   
   - :math:`start = (p_0, v_0, a_0)` (initial position, velocity, acceleration)
   - :math:`end = (p_f, v_f, a_f)` (final position, velocity, acceleration)
   - :math:`T` (duration in seconds)
   
   **Output:** Array of 6 coefficients :math:`[c_0, c_1, c_2, c_3, c_4, c_5]`

   The quintic polynomial is:
   
   .. math::
      p(t) = c_0 + c_1 t + c_2 t^2 + c_3 t^3 + c_4 t^4 + c_5 t^5

   1. Set coefficients from initial conditions:
   
      a. :math:`c_0 \gets p_0`
      
      b. :math:`c_1 \gets v_0`
      
      c. :math:`c_2 \gets a_0 / 2`
   
   2. Set up linear system for remaining coefficients:
   
      a. Boundary conditions at :math:`t = T`:
      
         - :math:`p(T) = p_f`
         - :math:`p'(T) = v_f`
         - :math:`p''(T) = a_f`
      
      b. Form matrix equation :math:`A \mathbf{x} = \mathbf{b}`:
      
         .. math::
            \begin{bmatrix} T^3 & T^4 & T^5 \\ 3T^2 & 4T^3 & 5T^4 \\ 6T & 12T^2 & 20T^3 \end{bmatrix} \begin{bmatrix} c_3 \\ c_4 \\ c_5 \end{bmatrix} = \begin{bmatrix} p_f - c_0 - c_1 T - c_2 T^2 \\ v_f - c_1 - 2 c_2 T \\ a_f - 2 c_2 \end{bmatrix}
   
   3. Solve: :math:`[c_3, c_4, c_5] \gets \text{np.linalg.solve}(A, b)`
   
   4. **return** :math:`[c_0, c_1, c_2, c_3, c_4, c_5]`


.. prf:algorithm:: quartic_coefficients
   :label: quartic-coefficients

   **Inputs:**
   
   - :math:`start = (p_0, v_0, a_0)` (initial position, velocity, acceleration)
   - :math:`end\_vel = (v_f, a_f)` (final velocity, acceleration — position unconstrained)
   - :math:`T` (duration in seconds)
   
   **Output:** Array of 5 coefficients :math:`[c_0, c_1, c_2, c_3, c_4]`

   The quartic polynomial is:
   
   .. math::
      p(t) = c_0 + c_1 t + c_2 t^2 + c_3 t^3 + c_4 t^4

   1. Set coefficients from initial conditions:
   
      a. :math:`c_0 \gets p_0`
      
      b. :math:`c_1 \gets v_0`
      
      c. :math:`c_2 \gets a_0 / 2`
   
   2. Set up linear system for remaining coefficients:
   
      a. Boundary conditions at :math:`t = T`:
      
         - :math:`p'(T) = v_f`
         - :math:`p''(T) = a_f`
      
      b. Form matrix equation :math:`A \mathbf{x} = \mathbf{b}`:
      
         .. math::
            \begin{bmatrix} 3T^2 & 4T^3 \\ 6T & 12T^2 \end{bmatrix} \begin{bmatrix} c_3 \\ c_4 \end{bmatrix} = \begin{bmatrix} v_f - c_1 - 2 c_2 T \\ a_f - 2 c_2 \end{bmatrix}
   
   3. Solve: :math:`[c_3, c_4] \gets \text{np.linalg.solve}(A, b)`
   
   4. **return** :math:`[c_0, c_1, c_2, c_3, c_4]`


.. prf:algorithm:: evaluate_polynomial
   :label: evaluate-polynomial

   **Inputs:**
   
   - :math:`coeffs` (array of polynomial coefficients :math:`[c_0, c_1, \ldots, c_n]`)
   - :math:`t` (time at which to evaluate)
   
   **Output:** Tuple of (position, velocity, acceleration)

   1. Compute position:
   
      a. :math:`p \gets \sum_{i=0}^{n} c_i \cdot t^i`
   
   2. Compute velocity (first derivative):
   
      a. :math:`v \gets \sum_{i=1}^{n} i \cdot c_i \cdot t^{i-1}`
   
   3. Compute acceleration (second derivative):
   
      a. :math:`a \gets \sum_{i=2}^{n} i \cdot (i-1) \cdot c_i \cdot t^{i-2}`
   
   4. **return** :math:`(p, v, a)`


.. prf:algorithm:: compute_jerk
   :label: compute-jerk

   **Inputs:**
   
   - :math:`coeffs` (array of polynomial coefficients)
   - :math:`t` (time at which to evaluate)
   
   **Output:** Jerk (third derivative) at time t

   1. :math:`j \gets \sum_{i=3}^{n} i \cdot (i-1) \cdot (i-2) \cdot c_i \cdot t^{i-3}`
   
   2. **return** :math:`j`


Task 3: Trajectory Planner
~~~~~~~~~~~~~~~~~~~~~~~~~~~

**File:** ``trajectory_planner.py``

Implement the main trajectory planning algorithm.


.. prf:algorithm:: TrajectoryPlanner.plan
   :label: trajectory-planner-plan

   **Inputs:**
   
   - :math:`ego` (ego state dict with x, y, theta, v, lane_id)
   - :math:`cmd` (BehavioralCommand with maneuver, target_lane, target_speed)
   - :math:`obstacles` (list of obstacle dicts)
   - :math:`path` (reference path as list of (x, y) waypoints)
   
   **Output:** (Trajectory, success: bool)

   1. **if** :math:`len(path) < 2` **then return** :math:`(\text{Trajectory}(), \text{False})`
   
   2. Create Frenet transform:
   
      a. :math:`frenet \gets \text{FrenetTransform}(path)`
   
   3. Convert ego state to Frenet:
   
      a. :math:`ego_f \gets frenet.\text{to_frenet}(ego.x, ego.y, ego.\theta, ego.v)`
   
   4. Determine target states from behavioral command:
   
      a. :math:`d_{target} \gets \text{compute_target_d}(cmd, ego)`
      
      b. :math:`v_{target} \gets cmd.target\_speed`
   
   5. Generate candidate trajectories:
   
      a. :math:`candidates \gets \text{generate_candidates}(ego_f, d_{target}, v_{target}, cmd, frenet)`
   
   6. Evaluate candidates:
   
      a. :math:`best \gets \text{Trajectory}()`
      
      b. :math:`best\_cost \gets \infty`
      
      c. **for** each :math:`traj` **in** :math:`candidates` **do**
      
         i. **if not** :math:`\text{check_feasibility}(traj)` **then continue**
         
         ii. :math:`traj.feasible \gets \text{True}`
         
         iii. **if not** :math:`\text{check_collision}(traj, obstacles)` **then continue**
         
         iv. :math:`traj.collision\_free \gets \text{True}`
         
         v. :math:`cost \gets \text{compute_cost}(traj, d_{target}, v_{target}, obstacles)`
         
         vi. **if** :math:`cost < best\_cost` **then**
         
             - :math:`best \gets traj`
             - :math:`best_cost \gets cost`
   
   7. **if** :math:`best.feasible` **and** :math:`best.collision\_free` **then**
   
      a. **return** :math:`(best, \text{True})`
   
   8. **else return** :math:`(\text{Trajectory}(), \text{False})`


.. prf:algorithm:: generate_candidates
   :label: generate-candidates

   **Inputs:**
   
   - :math:`ego_f` (ego state in Frenet coordinates)
   - :math:`d_{target}` (target lateral position)
   - :math:`v_{target}` (target velocity)
   - :math:`cmd` (behavioral command)
   - :math:`frenet` (FrenetTransform object)
   
   **Output:** List of candidate Trajectory objects

   1. :math:`candidates \gets []`
   
   2. Generate sample arrays:
   
      a. :math:`d\_samples \gets \text{linspace}(d_{target} - d\_range, d_{target} + d\_range, num\_d)`
      
      b. :math:`v\_samples \gets \text{linspace}(v_{target} - v\_range, v_{target} + v\_range, num\_v)`
      
      c. :math:`T\_samples \gets \text{linspace}(T_{min}, T_{max}, num\_T)`
   
   3. **for** each :math:`(d_f, v_f, T)` **in** :math:`d\_samples \times v\_samples \times T\_samples` **do**
   
      a. Generate lateral trajectory (quintic):
      
         i. :math:`d\_start \gets (ego_f.d, ego_f.\dot{d}, 0)`
         
         ii. :math:`d\_end \gets (d_f, 0, 0)` (end at rest laterally)
         
         iii. :math:`d\_coeffs \gets \text{quintic_coefficients}(d\_start, d\_end, T)`
      
      b. Generate longitudinal trajectory (quartic):
      
         i. :math:`s\_start \gets (ego_f.s, ego_f.\dot{s}, 0)`
         
         ii. :math:`s\_end\_vel \gets (v_f, 0)` (target velocity, zero final acceleration)
         
         iii. :math:`s\_coeffs \gets \text{quartic_coefficients}(s\_start, s\_end\_vel, T)`
      
      c. Sample trajectory at discrete times:
      
         i. :math:`points \gets []`
         
         ii. **for** :math:`t = 0` **to** :math:`T` **step** :math:`dt` **do**
         
             - :math:`s, \dot{s}, \ddot{s} \gets \text{evaluate_polynomial}(s\_coeffs, t)`
             - :math:`d, \dot{d}, \ddot{d} \gets \text{evaluate_polynomial}(d\_coeffs, t)`
             - :math:`cart \gets frenet.\text{to\_cartesian}(s, d, \dot{s}, \dot{d})`
             - :math:`\kappa \gets \text{compute_curvature}(\ddot{s}, \ddot{d}, cart.v)`
             - :math:`a \gets \text{compute_acceleration}(\ddot{s}, \ddot{d})`
             - :math:`points.\text{append}(\text{TrajectoryPoint}(t, cart.x, cart.y, cart.\theta, cart.v, \kappa, a))`
      
      d. :math:`traj \gets \text{Trajectory}(points, s\_traj=s\_coeffs, d\_traj=d\_coeffs)`
      
      e. :math:`candidates.\text{append}(traj)`
   
   4. **return** :math:`candidates`


.. prf:algorithm:: check_feasibility
   :label: check-feasibility

   **Inputs:**
   
   - :math:`traj` (Trajectory object)
   
   **Output:** True if all constraints satisfied, False otherwise

   1. **for** each :math:`point` **in** :math:`traj.points` **do**
   
      a. **if** :math:`point.v < 0` **or** :math:`point.v > v_{max}` **then return** False
      
      b. **if** :math:`point.a < a_{min}` **or** :math:`point.a > a_{max}` **then return** False
      
      c. **if** :math:`|point.\kappa| > \kappa_{max}` **then return** False
      
      d. :math:`a_{lat} \gets point.v^2 \cdot |point.\kappa|`
      
      e. **if** :math:`a_{lat} > a_{lat,max}` **then return** False
   
   2. **return** True


.. prf:algorithm:: check_collision
   :label: check-collision

   **Inputs:**
   
   - :math:`traj` (Trajectory object)
   - :math:`obstacles` (list of obstacle dicts with x, y, vx, vy, length, width)
   
   **Output:** True if collision-free, False if collision detected

   1. :math:`r_{ego} \gets \sqrt{(L_{ego}/2)^2 + (W_{ego}/2)^2} + margin`
   
   2. **for** each :math:`point` **in** :math:`traj.points` **do**
   
      a. **for** each :math:`obs` **in** :math:`obstacles` **do**
      
         i. Predict obstacle position at time :math:`point.t`:
         
            - :math:`obs_x \gets obs.x + obs.vx \cdot point.t`
            - :math:`obs_y \gets obs.y + obs.vy \cdot point.t`
         
         ii. :math:`r_{obs} \gets \sqrt{(obs.length/2)^2 + (obs.width/2)^2}`
         
         iii. :math:`dist \gets \sqrt{(point.x - obs_x)^2 + (point.y - obs_y)^2}`
         
         iv. **if** :math:`dist < r_{ego} + r_{obs}` **then return** False
   
   3. **return** True


.. prf:algorithm:: compute_cost
   :label: compute-cost

   **Inputs:**
   
   - :math:`traj` (Trajectory object with s_traj and d_traj polynomial coefficients)
   - :math:`d_{target}` (target lateral position)
   - :math:`v_{target}` (target velocity)
   - :math:`obstacles` (list of obstacles)
   
   **Output:** Total cost (lower is better)

   1. Compute jerk cost (integrated squared jerk):
   
      a. :math:`J_{jerk} \gets 0`
      
      b. **for** :math:`t = 0` **to** :math:`T` **step** :math:`dt` **do**
      
         i. :math:`j_s \gets \text{compute_jerk}(traj.s\_traj, t)`
         
         ii. :math:`j_d \gets \text{compute_jerk}(traj.d\_traj, t)`
         
         iii. :math:`J_{jerk} \gets J_{jerk} + (j_s^2 + j_d^2) \cdot dt`
   
   2. Compute lateral deviation cost:
   
      a. :math:`d_f, \_, \_ \gets \text{evaluate_polynomial}(traj.d\_traj, T)`
      
      b. :math:`J_{lat} \gets (d_f - d_{target})^2`
   
   3. Compute speed deviation cost:
   
      a. :math:`v_f \gets traj.points[-1].v`
      
      b. :math:`J_{speed} \gets (v_f - v_{target})^2`
   
   4. Compute time cost:
   
      a. :math:`J_{time} \gets traj.duration`
   
   5. Compute obstacle proximity cost:
   
      a. :math:`J_{obs} \gets 0`
      
      b. **for** each :math:`point` **in** :math:`traj.points` **do**
      
         i. **for** each :math:`obs` **in** :math:`obstacles` **do**
         
            - :math:`obs_x \gets obs.x + obs.vx \cdot point.t`
            - :math:`obs_y \gets obs.y + obs.vy \cdot point.t`
            - :math:`dist \gets \sqrt{(point.x - obs_x)^2 + (point.y - obs_y)^2}`
            - **if** :math:`dist < 20` **then** :math:`J_{obs} \gets J_{obs} + (20 - dist)^2`
   
   6. Combine costs with weights:
   
      a. :math:`J_{total} \gets w_{jerk} \cdot J_{jerk} + w_{lat} \cdot J_{lat} + w_{speed} \cdot J_{speed} + w_{time} \cdot J_{time} + w_{obs} \cdot J_{obs}`
   
   7. **return** :math:`J_{total}`


---------------------------------------------------------
6. Test Scenarios
---------------------------------------------------------

Your implementation will be tested on three scenarios that exercise different aspects of trajectory planning.

Scenario 1: Lane Keeping
~~~~~~~~~~~~~~~~~~~~~~~~~

**Description:** Straight highway segment with no obstacles. Generate trajectories that maintain lane center at constant speed.

**What It Tests:**

- Frenet coordinate transformation accuracy
- Polynomial trajectory generation
- Trajectory stays within lane bounds

**Success Criteria:**

- Lateral deviation from lane center < 0.3 m
- Speed maintained within ± 1 m/s of target
- Smooth trajectory (jerk < 2.5 m/s³)

Scenario 2: Vehicle Following
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Description:** Lead vehicle ahead with varying speed. Generate trajectories that maintain safe following distance.

**What It Tests:**

- Velocity adaptation in longitudinal trajectory
- Collision checking with moving obstacles
- Cost function balances speed and safety

**Success Criteria:**

- No collision with lead vehicle
- Minimum following distance > 15 m
- Smooth speed transitions

Scenario 3: Lane Change
~~~~~~~~~~~~~~~~~~~~~~~~

**Description:** Execute a lane change to pass a slow vehicle. Adjacent lane has sufficient gap.

**What It Tests:**

- Lateral trajectory generation for lane changes
- Quintic polynomial boundary conditions
- Collision checking in both lanes
- Feasibility constraints during maneuver

**Success Criteria:**

- Lane change completed within 6 seconds
- No collision with any vehicle
- Lateral acceleration < 3.0 m/s²
- Maximum curvature < 0.2 m⁻¹

---------------------------------------------------------
7. Getting Started
---------------------------------------------------------

Getting Started Checklist
~~~~~~~~~~~~~~~~~~~~~~~~~~

1. ☐ Assignment 3 (Behavioral Planner) completed
2. ☐ CARLA simulator installed and running
3. ☐ Starter code downloaded with your RWA3 implementation
4. ☐ Review L6 (Trajectory Planning) lecture materials
5. ☐ Understand Frenet coordinate system
6. ☐ Understand polynomial trajectory generation

Step 1: Implement Frenet Transform
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Open ``src/frenet_transform.py``
2. Implement ``compute_reference_path_properties()``
3. Implement ``find_closest_point_on_path()``
4. Implement ``cartesian_to_frenet()``
5. Implement ``frenet_to_cartesian()``
6. Test with ``python -m tests.test_frenet``

Step 2: Implement Polynomial Trajectories
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Open ``src/polynomial_trajectory.py``
2. Implement ``quintic_coefficients()``
3. Implement ``quartic_coefficients()``
4. Implement ``evaluate_polynomial()``
5. Implement ``compute_jerk()``
6. Test with ``python -m tests.test_polynomial``

Step 3: Implement Trajectory Planner
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Open ``src/trajectory_planner.py``
2. Implement ``_compute_target_d()``
3. Implement ``_generate_candidates()``
4. Implement ``_check_feasibility()``
5. Implement ``_check_collision()``
6. Implement ``_compute_cost()``
7. Implement ``plan()``

Step 4: Integration Testing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Start CARLA simulator
   ./CarlaUE4.sh
   
   # Run test scenarios
   python -m scenarios.scenario_runner --scenario highway --test lane_keep --visualize
   python -m scenarios.scenario_runner --scenario highway --test follow --visualize
   python -m scenarios.scenario_runner --scenario highway --test lane_change --visualize

Step 5: Parameter Tuning
~~~~~~~~~~~~~~~~~~~~~~~~~

1. Adjust cost weights in ``config/planner_config.yaml``
2. Test with different sampling parameters
3. Verify constraint satisfaction

---------------------------------------------------------
8. Provided Package Structure
---------------------------------------------------------

.. code-block:: text

   rwa3_final_starter/
   ├── src/
   │   ├── behavior_tree/
   │   │   ├── __init__.py              # BT framework classes (Provided)
   │   │   ├── bt_framework.py          # Re-exports (Provided)
   │   │   └── bt_nodes.py              # <-- RWA3
   │   ├── behavioral_planner.py        # <-- RWA3
   │   ├── carla_interface.py           # CARLA connection (Provided)
   │   ├── perception.py                # Perception module (Provided)
   │   ├── trajectory_planner.py        # <-- TODO: Implement
   │   ├── frenet_transform.py          # <-- TODO: Implement
   │   ├── polynomial_trajectory.py     # <-- TODO: Implement
   │   ├── controller.py                # Vehicle controller (Provided)
   │   └── visualization.py             # Visualization (Provided)
   ├── scenarios/
   │   ├── scenario_runner.py           # Main runner (Provided)
   │   └── highway_scenario.py          # Test scenarios (Provided)
   ├── tests/
   │   ├── test_bt_nodes.py             # Unit tests for BT nodes
   │   ├── test_frenet.py               # Frenet transform tests
   │   └── test_polynomial.py           # Polynomial tests
   ├── config/
   │   └── planner_config.yaml          # Configuration parameters
   └── requirements.txt                 # Python dependencies


.. note::

   **LLM Help:**
   
   Most of the CARLA scripts were generated using an LLM (claude.ai) and modified by the instructor
   

**Files you modify:**

- ✏️ ``src/frenet_transform.py`` — Coordinate transformation
- ✏️ ``src/polynomial_trajectory.py`` — Polynomial generation
- ✏️ ``src/trajectory_planner.py`` — Main planner

**Files from RWA3 (include your implementation):**

- 📎 ``src/behavior_tree/bt_nodes.py``
- 📎 ``src/behavioral_planner.py``

**Files you should NOT modify:**

- ❌ ``src/carla_interface.py``
- ❌ ``src/perception.py``
- ❌ ``src/controller.py``
- ❌ ``scenarios/*``

---------------------------------------------------------
9. Testing and Evaluation
---------------------------------------------------------

Run Unit Tests
~~~~~~~~~~~~~~~

.. code-block:: bash

   # Test Frenet transform
   python -m tests.test_frenet
   
   # Test polynomial trajectories
   python -m tests.test_polynomial

Evaluation Criteria
~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 30 70
   :header-rows: 1
   :class: compact-table

   * - **Metric**
     - **Requirement**
   * - **Frenet Transform**
     - Roundtrip error < 1.0 m
   * - **Polynomial Boundary Conditions**
     - Error < 1e-4 at t=0 and t=T
   * - **Feasibility Checking**
     - All constraints correctly verified
   * - **Collision Checking**
     - No false negatives (missed collisions)
   * - **Trajectory Quality**
     - Smooth, comfortable trajectories
   * - **Scenario Completion**
     - All scenarios pass without collision

---------------------------------------------------------
10. Submission Requirements
---------------------------------------------------------

What to Submit
~~~~~~~~~~~~~~

Submit a **single ZIP file** named ``groupX_final_project.zip`` containing:

.. code-block:: text

   groupX_final_project/
   ├── src/
   │   ├── behavior_tree/
   │   │   └── bt_nodes.py              # From RWA3
   │   ├── behavioral_planner.py        # From RWA3
   │   ├── frenet_transform.py          # Your implementation
   │   ├── polynomial_trajectory.py     # Your implementation
   │   └── trajectory_planner.py        # Your implementation
   ├── config/
   │   └── planner_config.yaml          # Your tuned parameters
   ├── results/
   │   ├── scenario_1_trajectory.png    # Trajectory plot
   │   ├── scenario_2_trajectory.png    # Trajectory plot
   │   ├── scenario_3_trajectory.png    # Trajectory plot
   │   ├── test_frenet_output.txt       # Unit test results
   │   └── test_polynomial_output.txt   # Unit test results
   └── report.pdf                       # Required report (3-5 pages)

Report Requirements
~~~~~~~~~~~~~~~~~~~~

**Length:** 3-5 pages maximum (excluding figures)

**Required Sections:**

1. **Frenet Transform:**
   
   - Explain your implementation approach
   - Discuss roundtrip accuracy
   - Any challenges encountered

2. **Polynomial Trajectories:**
   
   - Mathematical derivation of coefficient computation
   - Boundary condition verification

3. **Trajectory Planner:**
   
   - Sampling strategy and rationale
   - Cost function design and weight tuning
   - Feasibility and collision checking approach

4. **Results:**
   
   - Trajectory plots from each scenario
   - Discussion of trajectory quality
   - Performance metrics (computation time, success rate)

5. **Discussion:**
   
   - What worked well?
   - What was challenging?
   - How would you improve your implementation?

Submission Checklist
~~~~~~~~~~~~~~~~~~~~

Before submitting, ensure:

- ☐ Frenet transform implemented and tested
- ☐ Polynomial trajectory generation implemented and tested
- ☐ Trajectory planner implemented with all components
- ☐ Parameters tuned in ``planner_config.yaml``
- ☐ All three scenarios complete without collision
- ☐ Report complete with all sections (3-5 pages)
- ☐ Trajectory plots from each scenario included
- ☐ Code is well-commented
- ☐ ZIP file follows naming convention

---------------------------------------------------------
11. Grading Rubric
---------------------------------------------------------

**Total: 60 points**

**Frenet Transform (15 pts):**

- **cartesian_to_frenet (7 pts):**
  
  - 7 pts: Correct s, d, s_dot, d_dot computation
  - 5 pts: Position correct, velocity has issues
  - 3 pts: Partially functional
  - 0 pts: Not functional

- **frenet_to_cartesian (5 pts):**
  
  - 5 pts: Correct x, y, theta, v recovery
  - 3 pts: Position correct, heading/velocity issues
  - 0 pts: Not functional

- **Roundtrip accuracy (3 pts):**
  
  - 3 pts: Error < 0.5 m
  - 2 pts: Error < 1.0 m
  - 0 pts: Error > 1.0 m

**Polynomial Trajectories (15 pts):**

- **quintic_coefficients (6 pts):**
  
  - 6 pts: All 6 boundary conditions satisfied
  - 4 pts: Most conditions satisfied
  - 0 pts: Not functional

- **quartic_coefficients (4 pts):**
  
  - 4 pts: All 5 boundary conditions satisfied
  - 2 pts: Most conditions satisfied
  - 0 pts: Not functional

- **evaluate_polynomial (3 pts):**
  
  - 3 pts: Position, velocity, acceleration correct
  - 2 pts: Partial correctness
  - 0 pts: Not functional

- **compute_jerk (2 pts):**
  
  - 2 pts: Correct jerk computation
  - 0 pts: Not functional

**Trajectory Planner (20 pts):**

- **Candidate generation (5 pts):**
  
  - 5 pts: Correct sampling and trajectory creation
  - 3 pts: Trajectories generated with issues
  - 0 pts: Not functional

- **Feasibility checking (5 pts):**
  
  - 5 pts: All constraints correctly checked
  - 3 pts: Most constraints checked
  - 0 pts: Not functional

- **Collision checking (5 pts):**
  
  - 5 pts: Accurate collision detection with prediction
  - 3 pts: Static collision checking only
  - 0 pts: Not functional

- **Cost function (5 pts):**
  
  - 5 pts: All cost components, reasonable weighting
  - 3 pts: Partial cost implementation
  - 0 pts: Not functional

**Integration & Results (10 pts):**

- **Scenarios pass (6 pts):**
  
  - 6 pts: All 3 scenarios complete without collision
  - 4 pts: 2 scenarios pass
  - 2 pts: 1 scenario passes
  - 0 pts: No scenarios pass

- **Trajectory quality (4 pts):**
  
  - 4 pts: Smooth, comfortable trajectories
  - 2 pts: Some jerkiness or constraint violations
  - 0 pts: Poor quality trajectories

---------------------------------------------------------
12. Learning Outcomes
---------------------------------------------------------

By completing this project, you will:

- ✅ **Master Frenet coordinate transformation**
  
  - Convert between global and road-relative frames
  - Understand the benefits of road-aligned coordinates
  - Handle velocity decomposition

- ✅ **Implement polynomial trajectory generation**
  
  - Solve boundary value problems for trajectory planning
  - Understand quintic vs. quartic polynomials
  - Evaluate trajectories and their derivatives

- ✅ **Design trajectory optimization**
  
  - Sample-based trajectory generation
  - Multi-objective cost function design
  - Feasibility constraint checking

- ✅ **Implement collision avoidance**
  
  - Predict obstacle motion over time
  - Check trajectory-obstacle intersections
  - Balance safety and efficiency

- ✅ **Complete an end-to-end AV planning stack**
  
  - Integrate behavioral and trajectory planning
  - Interface with perception and control
  - Test in realistic simulation

**Connection to Course Material:**

This project implements concepts from L6 (Trajectory Planning):

- Frenet optimal trajectory method (Werling et al.)
- Polynomial trajectory representation
- Sample-based motion planning

Combined with RWA3, you've built a **complete autonomous driving planning system**!

---------------------------------------------------------
13. References
---------------------------------------------------------

**Primary References:**

- Werling, M., Ziegler, J., Kammel, S., & Thrun, S. (2010). *Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame.* IEEE International Conference on Robotics and Automation (ICRA).
- González, D., Pérez, J., Milanés, V., & Nashashibi, F. (2016). *A Review of Motion Planning Techniques for Automated Vehicles.* IEEE Transactions on Intelligent Transportation Systems.

**Documentation:**

- CARLA Simulator: https://carla.readthedocs.io/
- NumPy Linear Algebra: https://numpy.org/doc/stable/reference/routines.linalg.html

**Additional Reading:**

- Paden, B., Čáp, M., Yong, S. Z., Yershov, D., & Frazzoli, E. (2016). *A Survey of Motion Planning and Control Techniques Adopted in Self-Driving Vehicles.* IEEE Transactions on Intelligent Vehicles.
- Kelly, A. (2013). *Mobile Robotics: Mathematics, Models, and Methods.* Cambridge University Press.