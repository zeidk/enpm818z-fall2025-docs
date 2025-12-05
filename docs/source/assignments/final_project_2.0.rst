==================================================================
Final Project
==================================================================

:Title: Trajectory Planning in Frenet Coordinates

:Course: ENPM818Z — On-Road Automated Vehicles
:Topic: L6 — Trajectory Planning
:Assigned: December 1, 2025
:Due: December 12, 2025
:Total Points: 60 pts (+15 bonus)
:Language: Python

.. admonition:: Resources
   :class: resources

   - 🔗 `Starter package <https://github.com/zeidk/enpm818Z-fall-2025/tree/main/final_project_starter>`_
   
   - 📖 Lecture Materials: L6 (Trajectory Planning)

.. admonition:: Changelog

   **Version 2.0.0** (2025-12-01)

   - Refactored to use standalone simulator (no CARLA dependency)
   - CARLA integration now optional for bonus points
   - Added comprehensive unit tests and visualization

   **Version 1.0.0** (2025-11-30)

   - First version



.. note::

   **Project Structure:**
   
   This final project focuses on **Trajectory Planning** using Frenet coordinates and polynomial trajectories (Part 2 of the Planning Stack). This builds upon **Assignment 3** (Behavioral Planning with Behavior Trees). Together, these implement a complete hierarchical planning system for autonomous driving.
   
   **Prerequisite:** You must have completed Assignment 3 (Behavioral Planner) before starting this project.
   
   **Simulator Options:**
   
   - **Primary:** Standalone Python simulator with matplotlib visualization
   - **Bonus (+15 pts):** CARLA integration for realistic 3D visualization

---------------------------------------------------------
1. Objective
---------------------------------------------------------

Implement a **trajectory planner** that generates smooth, collision-free trajectories using the Frenet optimal trajectory method. Your implementation will integrate with the behavioral planner from Assignment 3 to create a complete autonomous driving planning system.

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

The trajectory planner receives commands from the behavioral planner and outputs trajectories for vehicle control.

Data Flow
~~~~~~~~~

.. list-table::
   :widths: 25 75
   :header-rows: 1
   :class: compact-table

   * - **Module**
     - **Description**
   * - **Simulator (Provided)**
     - Provides reference path, ego state, and traffic vehicles
   * - **Behavioral Planner (From RWA3)**
     - Outputs ``BehaviorCommand`` with maneuver type and targets
   * - **Trajectory Planner (You Implement)**
     - Generates smooth, collision-free trajectory
   * - **Visualization (Provided)**
     - Real-time display of trajectories and vehicle motion

What You Receive vs. Implement
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 40 15 15
   :header-rows: 1
   :class: compact-table

   * - **Module**
     - **Provided**
     - **You Implement**
   * - Simulator (``simulator.py``)
     - ✓
     - 
   * - Behavior Tree (``behavior_tree.py``)
     - 
     - ✓ (From RWA3)
   * - Frenet Transform (``frenet.py``)
     - 
     - ✓ (This Project)
   * - Polynomial Trajectory (``polynomial.py``)
     - 
     - ✓ (This Project)
   * - Cost Function (``cost.py``)
     - 
     - ✓ (This Project)
   * - Visualization (``visualizer.py``)
     - ✓
     - 
   * - Unit Tests (``test_*.py``)
     - ✓
     - 


Key Interfaces
~~~~~~~~~~~~~~

**Behavioral Planner → Trajectory Planner:**

.. code-block:: python

   @dataclass
   class BehaviorCommand:
       behavior: BehaviorType     # LANE_KEEP, FOLLOW_VEHICLE, LANE_CHANGE_LEFT, etc.
       target_d: float            # Target lateral offset (m)
       target_speed: float        # Target speed (m/s)
       T: float                   # Planning horizon (s)

**Trajectory Planner Output:**

.. code-block:: python

   @dataclass
   class TrajectoryState:
       t: float      # Time (seconds)
       s: float      # Longitudinal position (meters)
       d: float      # Lateral position (meters)
       s_dot: float  # Longitudinal velocity (m/s)
       d_dot: float  # Lateral velocity (m/s)
       s_ddot: float # Longitudinal acceleration (m/s²)
       d_ddot: float # Lateral acceleration (m/s²)
   
   @dataclass
   class Trajectory:
       states: List[TrajectoryState]  # Sequence of states
       T: float                        # Total duration
       cost: float                     # Total cost
       feasible: bool                  # Passed constraint checks

---------------------------------------------------------
4. Configuration Parameters
---------------------------------------------------------

The trajectory planner uses parameters that control generation, feasibility, and cost evaluation.

Vehicle Constraint Parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 30 15 55
   :header-rows: 1
   :class: compact-table

   * - **Parameter**
     - **Default**
     - **Description**
   * - ``max_velocity``
     - 40.0 m/s
     - Maximum allowable vehicle speed (~90 mph).
   * - ``max_acceleration``
     - 4.0 m/s²
     - Maximum comfortable acceleration.
   * - ``max_deceleration``
     - -8.0 m/s²
     - Maximum comfortable deceleration.
   * - ``max_lateral_accel``
     - 3.0 m/s²
     - Maximum lateral acceleration for comfort.
   * - ``max_jerk``
     - 10.0 m/s³
     - Maximum rate of change of acceleration.

Planning Parameters
~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 30 15 55
   :header-rows: 1
   :class: compact-table

   * - **Parameter**
     - **Default**
     - **Description**
   * - ``T_base``
     - 3.0 s
     - Base time horizon for trajectory generation.
   * - ``T_range``
     - 1.0 s
     - Range around T_base to sample (±T_range).
   * - ``dt``
     - 0.1 s
     - Time step for trajectory discretization.
   * - ``d_range``
     - 1.0 m
     - Range around target_d to sample.
   * - ``v_range``
     - 5.0 m/s
     - Range around target_speed to sample.

Cost Function Weights
~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 25 15 60
   :header-rows: 1
   :class: compact-table

   * - **Weight**
     - **Default**
     - **Description**
   * - ``w_jerk``
     - 1.0
     - Penalizes jerky motion (integral of squared jerk).
   * - ``w_time``
     - 1.0
     - Penalizes longer trajectories.
   * - ``w_d``
     - 1.0
     - Penalizes deviation from target lateral position.
   * - ``w_v``
     - 1.0
     - Penalizes deviation from target speed.
   * - ``w_accel``
     - 1.0
     - Penalizes high accelerations.

---------------------------------------------------------
5. Assignment Tasks
---------------------------------------------------------

You will implement three modules: **Frenet Transform**, **Polynomial Trajectory**, and **Cost Function**.

Task 1: Frenet Coordinate Transformation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**File:** ``frenet.py``

Implement functions to convert between Cartesian (x, y) and Frenet (s, d) coordinates.


.. prf:algorithm:: cartesian_to_frenet
   :label: cartesian-to-frenet

   **Inputs:**
   
   - :math:`x, y` (global position in meters)
   - :math:`ref\_path` (ReferencePath object with waypoints)
   
   **Output:** Tuple (s, d) - arc length and lateral offset

   1. Find closest point on reference path:
   
      a. :math:`min\_dist \gets \infty`
      
      b. :math:`closest\_idx \gets 0`
      
      c. **for** each point :math:`(px, py)` at index :math:`i` in path **do**
      
         i. :math:`dist \gets \sqrt{(x - px)^2 + (y - py)^2}`
         
         ii. **if** :math:`dist < min\_dist` **then** :math:`min\_dist \gets dist`, :math:`closest\_idx \gets i`
   
   2. Get arc length at closest point:
   
      a. :math:`s \gets ref\_path.s\_values[closest\_idx]`
   
   3. Compute lateral offset with sign:
   
      a. :math:`tangent \gets ref\_path.tangents[closest\_idx]`
      
      b. :math:`normal \gets (-tangent[1], tangent[0])` (rotate 90° left)
      
      c. :math:`dx \gets x - px`
      
      d. :math:`dy \gets y - py`
      
      e. :math:`d \gets dx \cdot normal[0] + dy \cdot normal[1]`
   
   4. **return** :math:`(s, d)`


.. prf:algorithm:: frenet_to_cartesian
   :label: frenet-to-cartesian

   **Inputs:**
   
   - :math:`s` (arc length along path in meters)
   - :math:`d` (lateral offset in meters, positive = left)
   - :math:`ref\_path` (ReferencePath object)
   
   **Output:** Tuple (x, y) - Cartesian coordinates

   1. Find point on path at arc length s (interpolate):
   
      a. :math:`idx \gets \text{searchsorted}(ref\_path.s\_values, s) - 1`
      
      b. :math:`idx \gets \text{clamp}(idx, 0, len(s\_values) - 2)`
      
      c. Interpolate position between path[idx] and path[idx+1]
   
   2. Get normal vector at this point:
   
      a. :math:`tangent \gets ref\_path.tangents[idx]`
      
      b. :math:`normal \gets (-tangent[1], tangent[0])`
   
   3. Compute Cartesian position:
   
      a. :math:`x \gets path\_x + d \cdot normal[0]`
      
      b. :math:`y \gets path\_y + d \cdot normal[1]`
   
   4. **return** :math:`(x, y)`


Task 2: Polynomial Trajectory Generation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**File:** ``polynomial.py``

Implement functions to compute polynomial coefficients and evaluate trajectories.


.. prf:algorithm:: quintic_coefficients
   :label: quintic-coefficients

   **Inputs:**
   
   - :math:`start = (p_0, v_0, a_0)` (initial position, velocity, acceleration)
   - :math:`end = (p_f, v_f, a_f)` (final position, velocity, acceleration)
   - :math:`T` (duration in seconds)
   
   **Output:** Array of 6 coefficients :math:`[a_0, a_1, a_2, a_3, a_4, a_5]`

   The quintic polynomial is:
   
   .. math::
      p(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3 + a_4 t^4 + a_5 t^5

   1. Set coefficients from initial conditions:
   
      a. :math:`a_0 \gets p_0`
      
      b. :math:`a_1 \gets v_0`
      
      c. :math:`a_2 \gets a_0 / 2`
   
   2. Set up linear system for remaining coefficients:
   
      a. Boundary conditions at :math:`t = T`:
      
         - :math:`p(T) = p_f`
         - :math:`p'(T) = v_f`
         - :math:`p''(T) = a_f`
      
      b. Form matrix equation :math:`A \mathbf{x} = \mathbf{b}`:
      
         .. math::
            \begin{bmatrix} T^3 & T^4 & T^5 \\ 3T^2 & 4T^3 & 5T^4 \\ 6T & 12T^2 & 20T^3 \end{bmatrix} \begin{bmatrix} a_3 \\ a_4 \\ a_5 \end{bmatrix} = \begin{bmatrix} p_f - a_0 - a_1 T - a_2 T^2 \\ v_f - a_1 - 2 a_2 T \\ a_f - 2 a_2 \end{bmatrix}
   
   3. Solve: :math:`[a_3, a_4, a_5] \gets \text{np.linalg.solve}(A, b)`
   
   4. **return** :math:`[a_0, a_1, a_2, a_3, a_4, a_5]`


.. prf:algorithm:: quartic_coefficients
   :label: quartic-coefficients

   **Inputs:**
   
   - :math:`start = (p_0, v_0, a_0)` (initial position, velocity, acceleration)
   - :math:`end\_vel = (v_f, a_f)` (final velocity, acceleration — position unconstrained)
   - :math:`T` (duration in seconds)
   
   **Output:** Array of 5 coefficients :math:`[a_0, a_1, a_2, a_3, a_4]`

   The quartic polynomial is:
   
   .. math::
      p(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3 + a_4 t^4

   1. Set coefficients from initial conditions:
   
      a. :math:`a_0 \gets p_0`
      
      b. :math:`a_1 \gets v_0`
      
      c. :math:`a_2 \gets a_0 / 2`
   
   2. Set up linear system for remaining coefficients:
   
      a. Boundary conditions at :math:`t = T`:
      
         - :math:`p'(T) = v_f`
         - :math:`p''(T) = a_f`
      
      b. Form matrix equation :math:`A \mathbf{x} = \mathbf{b}`:
      
         .. math::
            \begin{bmatrix} 3T^2 & 4T^3 \\ 6T & 12T^2 \end{bmatrix} \begin{bmatrix} a_3 \\ a_4 \end{bmatrix} = \begin{bmatrix} v_f - a_1 - 2 a_2 T \\ a_f - 2 a_2 \end{bmatrix}
   
   3. Solve: :math:`[a_3, a_4] \gets \text{np.linalg.solve}(A, b)`
   
   4. **return** :math:`[a_0, a_1, a_2, a_3, a_4]`


.. prf:algorithm:: generate_trajectory
   :label: generate-trajectory

   **Inputs:**
   
   - :math:`d\_coeffs` (lateral polynomial coefficients)
   - :math:`s\_coeffs` (longitudinal polynomial coefficients)
   - :math:`T` (duration)
   - :math:`dt` (time step)
   
   **Output:** Trajectory object with list of TrajectoryState

   1. :math:`states \gets []`
   
   2. **for** :math:`t = 0` **to** :math:`T` **step** :math:`dt` **do**
   
      a. Evaluate lateral polynomial:
      
         - :math:`d \gets \text{eval}(d\_coeffs, t)`
         - :math:`d\_dot \gets \text{eval\_deriv}(d\_coeffs, t, 1)`
         - :math:`d\_ddot \gets \text{eval\_deriv}(d\_coeffs, t, 2)`
      
      b. Evaluate longitudinal polynomial:
      
         - :math:`s \gets \text{eval}(s\_coeffs, t)`
         - :math:`s\_dot \gets \text{eval\_deriv}(s\_coeffs, t, 1)`
         - :math:`s\_ddot \gets \text{eval\_deriv}(s\_coeffs, t, 2)`
      
      c. :math:`states.\text{append}(\text{TrajectoryState}(t, s, d, s\_dot, d\_dot, s\_ddot, d\_ddot))`
   
   3. **return** Trajectory(states, T, d_coeffs, s_coeffs)


Task 3: Cost Function and Feasibility
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**File:** ``cost.py``

Implement functions to evaluate trajectory cost and check feasibility.


.. prf:algorithm:: compute_total_cost
   :label: compute-total-cost

   **Inputs:**
   
   - :math:`traj` (Trajectory object)
   - :math:`target\_d` (target lateral position)
   - :math:`target\_speed` (target velocity)
   - :math:`weights` (CostWeights object)
   
   **Output:** Total cost (lower is better)

   1. Compute jerk cost (integrated squared jerk):
   
      a. :math:`J_{jerk} \gets 0`
      
      b. **for** each state in trajectory **do**
      
         i. Compute :math:`j_s` (third derivative of s polynomial)
         
         ii. Compute :math:`j_d` (third derivative of d polynomial)
         
         iii. :math:`J_{jerk} \gets J_{jerk} + (j_s^2 + j_d^2) \cdot dt`
   
   2. Compute time cost:
   
      a. :math:`J_{time} \gets traj.T`
   
   3. Compute lateral deviation cost:
   
      a. :math:`d_f \gets traj.states[-1].d`
      
      b. :math:`J_{lat} \gets (d_f - target\_d)^2`
   
   4. Compute speed deviation cost:
   
      a. :math:`v_f \gets traj.states[-1].s\_dot`
      
      b. :math:`J_{vel} \gets (v_f - target\_speed)^2`
   
   5. Compute total weighted cost:
   
      a. :math:`cost \gets w_{jerk} \cdot J_{jerk} + w_{time} \cdot J_{time} + w_d \cdot J_{lat} + w_v \cdot J_{vel}`
   
   6. **return** :math:`cost`


.. prf:algorithm:: check_feasibility
   :label: check-feasibility

   **Inputs:**
   
   - :math:`traj` (Trajectory object)
   - :math:`limits` (FeasibilityLimits object)
   
   **Output:** True if all constraints satisfied, False otherwise

   1. **for** each :math:`state` **in** :math:`traj.states` **do**
   
      a. Check velocity bounds:
      
         - **if** :math:`state.s\_dot < 0` **or** :math:`state.s\_dot > v_{max}` **then return** False
      
      b. Check acceleration bounds:
      
         - :math:`a_{total} \gets \sqrt{state.s\_ddot^2 + state.d\_ddot^2}`
         - **if** :math:`state.s\_ddot < a_{min}` **or** :math:`state.s\_ddot > a_{max}` **then return** False
      
      c. Check lateral acceleration:
      
         - :math:`a_{lat} \gets |state.d\_ddot|`
         - **if** :math:`a_{lat} > a_{lat,max}` **then return** False
   
   2. Check jerk bounds (compute from polynomial):
   
      a. **for** each time step **do**
      
         - Compute jerk from third derivative
         - **if** :math:`|jerk| > j_{max}` **then return** False
   
   3. **return** True


.. prf:algorithm:: select_best_trajectory
   :label: select-best-trajectory

   **Inputs:**
   
   - :math:`candidates` (list of Trajectory objects)
   - :math:`target\_d, target\_speed` (targets from behavior planner)
   - :math:`weights, limits` (cost weights and feasibility limits)
   
   **Output:** Best feasible trajectory, or None

   1. :math:`best \gets \text{None}`
   
   2. :math:`best\_cost \gets \infty`
   
   3. **for** each :math:`traj` **in** :math:`candidates` **do**
   
      a. **if not** check_feasibility(traj, limits) **then continue**
      
      b. :math:`traj.feasible \gets \text{True}`
      
      c. :math:`cost \gets \text{compute\_total\_cost}(traj, target\_d, target\_speed, weights)`
      
      d. :math:`traj.cost \gets cost`
      
      e. **if** :math:`cost < best\_cost` **then**
      
         - :math:`best \gets traj`
         - :math:`best\_cost \gets cost`
   
   4. **return** :math:`best`


---------------------------------------------------------
6. Test Scenarios
---------------------------------------------------------

Your implementation will be tested on three scenarios of increasing complexity.

Scenario 1: Cruising (Empty Road)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Description:** Maintain lane at constant speed on empty highway.

**What It Tests:**

- Frenet coordinate transformation
- Polynomial trajectory generation
- Basic feasibility checking

**Expected Behavior:**

- Vehicle maintains d ≈ 0 (lane center)
- Vehicle reaches target speed (31 m/s)
- Smooth acceleration profile

Scenario 2: Following
~~~~~~~~~~~~~~~~~~~~~~

**Description:** Follow a lead vehicle traveling at constant speed.

**What It Tests:**

- Speed adaptation in trajectory
- Collision checking with lead vehicle
- Smooth following behavior

**Expected Behavior:**

- Vehicle matches lead vehicle speed
- Maintains safe following distance
- No collision with lead vehicle

Scenario 3: Overtaking
~~~~~~~~~~~~~~~~~~~~~~~

**Description:** Overtake a slow vehicle by changing to left lane.

**What It Tests:**

- Lane change trajectory generation
- Collision checking during lane change
- Return to original lane

**Expected Behavior:**

- Smooth lateral transition (d: 0 → 3.5 → 0)
- No collision during overtake
- Returns to center lane after passing

---------------------------------------------------------
7. Getting Started
---------------------------------------------------------

Getting Started Checklist
~~~~~~~~~~~~~~~~~~~~~~~~~~

1. ☐ Completed Assignment 3 (Behavioral Planner)
2. ☐ Starter code downloaded and environment configured
3. ☐ Can run ``python simulator.py --no-viz``
4. ☐ Understand Frenet coordinates from L6 lecture
5. ☐ Understand polynomial trajectory generation

Step 1: Set Up Environment
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Use same environment from RWA3
   source venv/bin/activate
   
   # Copy your behavior_tree.py from RWA3
   cp ../rwa3/behavior_tree.py .

Step 2: Implement Frenet Transform
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Open ``frenet.py``
2. Implement ``cartesian_to_frenet()``
3. Implement ``frenet_to_cartesian()``
4. Test roundtrip accuracy: ``python test_frenet.py``

Step 3: Implement Polynomial Generation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Open ``polynomial.py``
2. Implement ``quintic_coefficients()``
3. Implement ``quartic_coefficients()``
4. Implement ``generate_trajectory()``
5. Test: ``python test_polynomial.py``

Step 4: Implement Cost Function
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Open ``cost.py``
2. Implement ``compute_total_cost()``
3. Implement ``check_feasibility()``
4. Implement ``select_best_trajectory()``
5. Test: ``python test_cost.py``

Step 5: Integration Testing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Run with visualization
   python simulator.py --scenario empty
   python simulator.py --scenario follow
   python simulator.py --scenario overtake
   
   # Run text-only mode
   python simulator.py --no-viz --scenario overtake --duration 40

---------------------------------------------------------
8. Provided Package Structure
---------------------------------------------------------

.. code-block:: text

   final_project_starter/
   ├── frenet.py                 # <-- TODO: Implement Frenet transforms
   ├── polynomial.py             # <-- TODO: Implement polynomial generation
   ├── cost.py                   # <-- TODO: Implement cost and feasibility
   ├── behavior_tree.py          # Copy from RWA3
   ├── simulator.py              # Standalone simulator (Provided)
   ├── visualizer.py             # Matplotlib visualization (Provided)
   ├── test_frenet.py            # Unit tests (Provided)
   ├── test_polynomial.py        # Unit tests (Provided)
   ├── test_cost.py              # Unit tests (Provided)
   ├── requirements.txt          # Python dependencies
   └── README.md                 # Instructions

**Files you implement:**

- ✏️ ``frenet.py`` — Frenet coordinate transforms
- ✏️ ``polynomial.py`` — Polynomial trajectory generation
- ✏️ ``cost.py`` — Cost function and feasibility checking

**Files you copy from RWA3:**

- 📋 ``behavior_tree.py`` — Your behavioral planner

**Files you should NOT modify:**

- ❌ ``simulator.py`` — Provided simulator
- ❌ ``visualizer.py`` — Provided visualization
- ❌ ``test_*.py`` — Provided unit tests

---------------------------------------------------------
9. Testing and Evaluation
---------------------------------------------------------

Run Unit Tests
~~~~~~~~~~~~~~~

.. code-block:: bash

   # Test each module
   python test_frenet.py
   python test_polynomial.py
   python test_cost.py
   
   # Run all tests
   python -m pytest test_*.py -v

Evaluation Criteria
~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 30 70
   :header-rows: 1
   :class: compact-table

   * - **Metric**
     - **Requirement**
   * - **Frenet Transform**
     - Roundtrip error < 0.5 m
   * - **Polynomial Generation**
     - All boundary conditions satisfied
   * - **Feasibility Checking**
     - All constraints correctly verified
   * - **Cost Function**
     - Correct cost computation and trajectory selection
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
   ├── frenet.py                     # Your implementation
   ├── polynomial.py                 # Your implementation
   ├── cost.py                       # Your implementation
   ├── behavior_tree.py              # From RWA3
   ├── results/
   │   ├── scenario_empty.png        # Screenshot
   │   ├── scenario_follow.png       # Screenshot
   │   ├── scenario_overtake.png     # Screenshot
   │   └── test_output.txt           # Unit test results
   └── report.pdf                    # Required report (3-5 pages)

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

3. **Cost Function:**
   
   - Cost components and their purpose
   - Weight tuning process

4. **Results:**
   
   - Trajectory plots from each scenario
   - Discussion of trajectory quality

5. **Discussion:**
   
   - What worked well?
   - What was challenging?
   - How would you improve your implementation?

Submission Checklist
~~~~~~~~~~~~~~~~~~~~

Before submitting, ensure:

- ☐ Frenet transform implemented and tested
- ☐ Polynomial trajectory generation implemented and tested
- ☐ Cost function and feasibility checking implemented
- ☐ All three scenarios complete without collision
- ☐ Report complete with all sections (3-5 pages)
- ☐ Screenshots from each scenario included
- ☐ Code is well-commented
- ☐ ZIP file follows naming convention

---------------------------------------------------------
11. Grading Rubric
---------------------------------------------------------

**Total: 60 points**

**Frenet Transform (15 pts):**

- **cartesian_to_frenet (7 pts):**
  
  - 7 pts: Correct s, d computation
  - 5 pts: s correct, d has sign issues
  - 3 pts: Partially functional
  - 0 pts: Not functional

- **frenet_to_cartesian (5 pts):**
  
  - 5 pts: Correct x, y recovery
  - 3 pts: Position approximately correct
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

- **generate_trajectory (5 pts):**
  
  - 5 pts: Correct state evaluation at all time steps
  - 3 pts: Partial correctness
  - 0 pts: Not functional

**Cost Function (15 pts):**

- **compute_total_cost (6 pts):**
  
  - 6 pts: All cost components correctly computed
  - 4 pts: Most components correct
  - 0 pts: Not functional

- **check_feasibility (5 pts):**
  
  - 5 pts: All constraints correctly checked
  - 3 pts: Most constraints checked
  - 0 pts: Not functional

- **select_best_trajectory (4 pts):**
  
  - 4 pts: Correctly selects lowest-cost feasible trajectory
  - 2 pts: Selection works with issues
  - 0 pts: Not functional

**Integration & Results (15 pts):**

- **Scenarios pass (9 pts):**
  
  - 9 pts: All 3 scenarios complete without collision
  - 6 pts: 2 scenarios pass
  - 3 pts: 1 scenario passes
  - 0 pts: No scenarios pass

- **Trajectory quality (3 pts):**
  
  - 3 pts: Smooth, comfortable trajectories
  - 2 pts: Some jerkiness
  - 0 pts: Poor quality

- **Report (3 pts):**
  
  - 3 pts: Complete, well-written report
  - 2 pts: Partial report
  - 0 pts: Missing report

---------------------------------------------------------
12. Bonus: CARLA Integration (+15 pts)
---------------------------------------------------------

For additional credit, integrate your trajectory planner with the CARLA simulator.

Requirements
~~~~~~~~~~~~~

1. Install CARLA 0.9.13 or later
2. Implement ``carla_interface.py`` to connect to CARLA
3. Convert CARLA waypoints to reference path
4. Visualize trajectories in CARLA
5. Run all three scenarios in CARLA

Bonus Grading
~~~~~~~~~~~~~~

- **+5 pts:** CARLA connection works, ego vehicle follows trajectories
- **+5 pts:** Traffic vehicles correctly simulated and avoided
- **+5 pts:** All scenarios complete successfully with video evidence

**Submission:** Include a ``carla/`` folder with your CARLA integration code and video recordings of each scenario (MP4, max 60 seconds each).

---------------------------------------------------------
13. Learning Outcomes
---------------------------------------------------------

By completing this project, you will:

- ✅ **Master Frenet coordinate transformation**
  
  - Convert between global and road-relative frames
  - Understand the benefits of road-aligned coordinates

- ✅ **Implement polynomial trajectory generation**
  
  - Solve boundary value problems for trajectory planning
  - Understand quintic vs. quartic polynomials
  - Evaluate trajectories and their derivatives

- ✅ **Design trajectory optimization**
  
  - Sample-based trajectory generation
  - Multi-objective cost function design
  - Feasibility constraint checking

- ✅ **Complete an end-to-end AV planning stack**
  
  - Integrate behavioral and trajectory planning
  - Test in realistic simulation

**Connection to Course Material:**

This project implements concepts from L6 (Trajectory Planning):

- Frenet optimal trajectory method (Werling et al.)
- Polynomial trajectory representation
- Sample-based motion planning

Combined with RWA3, you've built a **complete autonomous driving planning system**!

---------------------------------------------------------
14. References
---------------------------------------------------------

**Primary References:**

- Werling, M., Ziegler, J., Kammel, S., & Thrun, S. (2010). *Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame.* IEEE International Conference on Robotics and Automation (ICRA).
- González, D., Pérez, J., Milanés, V., & Nashashibi, F. (2016). *A Review of Motion Planning Techniques for Automated Vehicles.* IEEE Transactions on Intelligent Transportation Systems.

**Additional Reading:**

- Paden, B., Čáp, M., Yong, S. Z., Yershov, D., & Frazzoli, E. (2016). *A Survey of Motion Planning and Control Techniques Adopted in Self-Driving Vehicles.* IEEE Transactions on Intelligent Vehicles.
