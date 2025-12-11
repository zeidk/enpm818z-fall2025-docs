==================================================================
Final Project
==================================================================

:Title: Trajectory Planning in Frenet Coordinates

:Course: ENPM818Z — On-Road Automated Vehicles
:Topic: L6 — Trajectory Planning
:Assigned: December 1, 2025
:Due: December 13, 2025
:Total Points: 60 pts (+15 bonus)
:Language: Python

.. admonition:: Resources
   :class: resources

   - 🔗 `Starter package <https://github.com/zeidk/enpm818Z-fall-2025/tree/main/final_project_starter>`_
   
   - 📖 Lecture Materials: L6 (Trajectory Planning)

.. admonition:: Changelog

   **Version 2.2.0** (2025-12-05)

   - Updated visualizer with behavior summary and polynomial coefficients panels
   - Added "Return to Center" behavior for proper lane change display
   - Improved trajectory planning parameters for smoother motion
   - Added behavior summary display at end of simulation
   - Fixed follow scenario vehicle spacing

   **Version 2.1.0** (2025-12-05)

   - Added detailed RWA3 integration instructions
   - Added evaluation guide for standalone mode
   - Clarified file dependencies and module connections

   **Version 2.0.0** (2025-12-01)

   - Refactored to use standalone simulator (no CARLA dependency)
   - Added comprehensive unit tests and visualization

   **Version 1.0.0** (2025-11-30)

   - First version



.. note::

   **Project Structure:**
   
   This final project focuses on **Trajectory Planning** using Frenet coordinates and polynomial trajectories (Part 2 of the Planning Stack). The behavioral planner from **Assignment 3** is included in the starter package, creating a complete hierarchical planning system for autonomous driving.
   
   **Simulator Options:**
   
   - **Primary:** Standalone Python simulator with matplotlib visualization (60 pts)

---------------------------------------------------------
1. Objective
---------------------------------------------------------

Implement a **trajectory planner** that generates smooth, collision-free trajectories using the Frenet optimal trajectory method. Your implementation will integrate with the behavioral planner from Assignment 3 to create a complete autonomous driving planning system.

**Learning Objectives:**

- Implement Frenet coordinate transformation between Cartesian and road-relative frames
- Generate polynomial trajectories that satisfy boundary conditions
- Design and tune cost functions for trajectory evaluation
- Implement feasibility checking for vehicle dynamic constraints
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

---------------------------------------------------------
3. System Architecture
---------------------------------------------------------

Complete Planning Stack
~~~~~~~~~~~~~~~~~~~~~~~~

The final project integrates with RWA3 to create a complete planning system:

.. only:: html

   .. figure:: /_static/final/final_stack_light.png
      :alt: Final project stack
      :align: center
      :width: 90%
      :class: only-light

   .. figure:: /_static/final/final_stack_dark.png
      :alt: Final project stack
      :align: center
      :width: 90%
      :class: only-dark


Data Flow Summary
~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 20 35 45
   :header-rows: 1
   :class: compact-table

   * - **Stage**
     - **Input**
     - **Output**
   * - Perception
     - Raw sensor data
     - ``EnvironmentState``
   * - Behavioral Planning (RWA3)
     - ``EnvironmentState``
     - ``BehaviorCommand``
   * - Trajectory Planning (Final Project)
     - ``BehaviorCommand`` + current state
     - ``Trajectory``
   * - Control
     - ``Trajectory``
     - Steering, throttle, brake

---------------------------------------------------------
4. Integrating RWA3 with Final Project
---------------------------------------------------------

This section explains exactly how to connect your RWA3 behavioral planner to the final project trajectory planner.

Required Files from RWA3
~~~~~~~~~~~~~~~~~~~~~~~~~

Copy these files from your completed RWA3 solution into the final project folder:

.. list-table::
   :widths: 25 75
   :header-rows: 1
   :class: compact-table

   * - **File**
     - **Purpose**
   * - ``bt_framework.py``
     - Base classes (Node, Sequence, Selector, Status, blackboard)
   * - ``bt_nodes.py``
     - Your implemented condition/action nodes
   * - ``behavior_tree.py``
     - Your ``BehaviorPlanner`` class with ``_build_tree()``

Step-by-Step Integration
~~~~~~~~~~~~~~~~~~~~~~~~~

**Step 1: Copy RWA3 Files**

.. code-block:: bash

   # From your RWA3 solution directory
   cp bt_framework.py bt_nodes.py behavior_tree.py /path/to/final_project/

**Step 2: Verify File Structure**

Your final project folder should contain:

.. code-block:: text

   final_project/
   ├── bt_framework.py           # From RWA3
   ├── bt_nodes.py               # From RWA3 (your implementation)
   ├── behavior_tree.py          # From RWA3 (your implementation)
   ├── frenet.py                 # TODO: Implement
   ├── polynomial.py             # TODO: Implement
   ├── cost.py                   # TODO: Implement
   ├── simulator.py              # Provided
   ├── visualizer.py             # Provided
   ├── test_frenet.py            # Provided
   ├── test_polynomial.py        # Provided
   ├── test_cost.py              # Provided
   └── requirements.txt          # Provided

**Step 3: Verify RWA3 Integration**

Run this test to ensure your behavioral planner works:

.. code-block:: bash

   # Test behavioral planner standalone
   python3 -c "
   from behavior_tree import BehaviorPlanner
   from bt_framework import EnvironmentState
   
   planner = BehaviorPlanner()
   env = EnvironmentState()
   env.ego_speed = 25.0
   env.speed_limit = 31.0
   env.vehicle_ahead = False
   
   cmd = planner.get_command(env)
   print(f'Behavior: {cmd.behavior.value}')
   print(f'Target d: {cmd.target_d}')
   print(f'Target speed: {cmd.target_speed}')
   "

Expected output:

.. code-block:: text

   Behavior: lane_keep
   Target d: 0.0
   Target speed: 31.0

How the Integration Works
~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``simulator.py`` file orchestrates the integration:

.. code-block:: python

   # In simulator.py - main loop
   
   from behavior_tree import BehaviorPlanner
   from frenet import cartesian_to_frenet, frenet_to_cartesian
   from polynomial import generate_candidate_trajectories
   from cost import select_best_trajectory
   
   # Initialize
   sim = Simulator()
   planner = BehaviorPlanner()  # Your RWA3 implementation
   
   while running:
       # 1. Get environment state (from simulator)
       env = sim.get_environment_state()
       
       # 2. Behavioral planning (YOUR RWA3 CODE)
       command = planner.get_command(env)
       # Returns: BehaviorCommand(behavior, target_d, target_speed, T)
       
       # 3. Trajectory planning (YOUR FINAL PROJECT CODE)
       # Convert current state to Frenet
       s, d = cartesian_to_frenet(ego.x, ego.y, ref_path)
       
       # Generate candidate trajectories
       candidates = generate_candidate_trajectories(
           current_state={'s': s, 'd': d, 's_dot': ego.speed, ...},
           target_d=command.target_d,
           target_speed=command.target_speed,
           T_base=command.T
       )
       
       # Select best trajectory
       best = select_best_trajectory(candidates, target_d, target_speed)
       
       # 4. Execute trajectory
       sim.step(command)  # Updates ego position along trajectory

Key Interface: BehaviorCommand
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``BehaviorCommand`` from RWA3 drives trajectory generation:

.. code-block:: python

   @dataclass
   class BehaviorCommand:
       behavior: BehaviorType     # What maneuver to execute
       target_d: float            # Target lateral offset (meters)
       target_speed: float        # Target speed (m/s)
       T: float                   # Planning horizon (seconds)

**How each behavior maps to trajectory parameters:**

.. list-table::
   :widths: 25 20 20 35
   :header-rows: 1
   :class: compact-table

   * - **Behavior**
     - **target_d**
     - **target_speed**
     - **Trajectory Effect**
   * - ``LANE_KEEP``
     - 0.0
     - speed_limit
     - Stay centered, cruise at limit
   * - ``FOLLOW_VEHICLE``
     - 0.0
     - lead_speed - 1
     - Stay centered, match lead speed
   * - ``LANE_CHANGE_LEFT``
     - +lane_width
     - speed_limit
     - Move left one lane width
   * - ``LANE_CHANGE_RIGHT``
     - -lane_width
     - speed_limit
     - Move right one lane width

---------------------------------------------------------
5. What You Implement
---------------------------------------------------------

You must implement three Python modules for trajectory planning.

frenet.py (15 pts)
~~~~~~~~~~~~~~~~~~~

**cartesian_to_frenet** — Convert (x, y) to (s, d):

.. code-block:: python

   def cartesian_to_frenet(x: float, y: float, ref_path: ReferencePath) -> Tuple[float, float]:
       """
       Algorithm:
       1. Find closest point on reference path (minimize distance)
       2. Get arc length s at that point
       3. Compute d as dot product of offset vector with normal
       4. Return (s, d)
       """

**frenet_to_cartesian** — Convert (s, d) to (x, y):

.. code-block:: python

   def frenet_to_cartesian(s: float, d: float, ref_path: ReferencePath) -> Tuple[float, float]:
       """
       Algorithm:
       1. Find position on path at arc length s (interpolate if needed)
       2. Get normal vector at that position
       3. Compute x = path_x + d * normal_x
       4. Compute y = path_y + d * normal_y
       5. Return (x, y)
       """

polynomial.py (15 pts)
~~~~~~~~~~~~~~~~~~~~~~~

**quintic_coefficients** — Compute 6 coefficients for lateral trajectory:

.. code-block:: python

   def quintic_coefficients(start, end, T) -> np.ndarray:
       """
       Quintic: p(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
       
       Boundary conditions:
       - p(0) = p0, p'(0) = v0, p''(0) = a0
       - p(T) = pf, p'(T) = vf, p''(T) = af
       
       Returns [a0, a1, a2, a3, a4, a5]
       """

**quartic_coefficients** — Compute 5 coefficients for longitudinal trajectory:

.. code-block:: python

   def quartic_coefficients(start, end_vel, T) -> np.ndarray:
       """
       Quartic: p(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴
       
       Boundary conditions (position-free at end):
       - p(0) = p0, p'(0) = v0, p''(0) = a0
       - p'(T) = vf, p''(T) = af
       
       Returns [a0, a1, a2, a3, a4]
       """

**evaluate_polynomial** — Evaluate polynomial and derivatives:

.. code-block:: python

   def evaluate_polynomial(coeffs, t) -> Tuple[float, float, float]:
       """Returns (position, velocity, acceleration) at time t."""

**generate_trajectory** — Sample polynomial to create trajectory:

.. code-block:: python

   def generate_trajectory(d_coeffs, s_coeffs, T, dt) -> Trajectory:
       """Sample polynomials at dt intervals, return Trajectory object."""

cost.py (15 pts)
~~~~~~~~~~~~~~~~~

**compute_total_cost** — Evaluate trajectory quality:

.. code-block:: python

   def compute_total_cost(traj, target_d, target_speed, weights) -> float:
       """
       Cost = w_jerk * J_jerk + w_time * T + w_d * (d_final - target_d)² 
            + w_v * (v_final - target_v)² + w_accel * J_accel
       """

**check_feasibility** — Verify vehicle constraints:

.. code-block:: python

   def check_feasibility(traj, limits) -> bool:
       """
       Check all states satisfy:
       - 0 <= velocity <= max_velocity
       - max_decel <= acceleration <= max_accel
       - |lateral_accel| <= max_lateral_accel
       - |jerk| <= max_jerk
       """

**select_best_trajectory** — Choose optimal trajectory:

.. code-block:: python

   def select_best_trajectory(candidates, target_d, target_speed) -> Trajectory:
       """Return lowest-cost feasible trajectory, or None."""

---------------------------------------------------------
6. Evaluation: Standalone Simulator (60 pts)
---------------------------------------------------------

This section covers how to test and evaluate your implementation using the standalone Python simulator.

Running Unit Tests
~~~~~~~~~~~~~~~~~~~

Test each module individually before running the full simulation:

.. code-block:: bash

   # Navigate to your project folder
   cd final_project/
   
   # Test Frenet transformations (6 tests)
   python3 test_frenet.py
   
   # Test polynomial generation (9 tests)
   python3 test_polynomial.py
   
   # Test cost functions (9 tests)
   python3 test_cost.py

**Expected output for each:**

.. code-block:: text

   ============================================================
   RESULTS: X passed, 0 failed
   ============================================================
   
   ✅ All tests passed!

Running Module Self-Tests
~~~~~~~~~~~~~~~~~~~~~~~~~~

Each implementation file has built-in tests that you can run directly:

.. code-block:: bash

   # Test frenet.py implementation
   python3 frenet.py

**frenet.py** tests Frenet coordinate transformations:

- Creates a straight reference path (100m)
- Tests ``cartesian_to_frenet()`` with points on, left, and right of centerline
- Tests ``frenet_to_cartesian()`` for inverse transformation
- Verifies roundtrip accuracy (Cartesian → Frenet → Cartesian < 0.5m error)

**Expected output:**

.. code-block:: text

   Testing Frenet Coordinate Transformation...

   1. Testing with straight path (x = 0 to 100):
      Path length: 100.0m
      Number of waypoints: 11

      Point on centerline: (50.0, 0.0)
      Frenet: s=50.0m, d=0.00m
      Expected: s=50.0, d=0.0 ✓

      Point left of centerline: (50.0, 3.5)
      Frenet: s=50.0m, d=3.50m
      Expected: s=50.0, d=3.5 ✓

      Point right of centerline: (50.0, -3.5)
      Frenet: s=50.0m, d=-3.50m
      Expected: s=50.0, d=-3.5 ✓

   2. Testing frenet_to_cartesian:
      Frenet: s=50.0, d=0.0
      Cartesian: (50.0, 0.0)
      Expected: (50.0, 0.0) ✓

      Frenet: s=50.0, d=3.5
      Cartesian: (50.0, 3.5)
      Expected: (50.0, 3.5) ✓

   3. Testing roundtrip accuracy:
      (25.0, 1.5) -> (s=25.0, d=1.50) -> (25.0, 1.5), error=0.000m ✓
      (50.0, -2.0) -> (s=50.0, d=-2.00) -> (50.0, -2.0), error=0.000m ✓
      (75.0, 3.0) -> (s=75.0, d=3.00) -> (75.0, 3.0), error=0.000m ✓

      Maximum roundtrip error: 0.000m
      Requirement: < 0.5m ✓

   ==================================================
   All tests complete.

.. code-block:: bash

   # Test polynomial.py implementation
   python3 polynomial.py

**polynomial.py** tests polynomial trajectory generation:

- Tests ``quintic_coefficients()`` for lane change (d: 0→3.5m in 4s)
- Verifies all 6 boundary conditions (position, velocity, acceleration at start/end)
- Tests ``quartic_coefficients()`` for speed change (v: 20→25 m/s in 3s)
- Verifies all 5 boundary conditions
- Tests ``generate_trajectory()`` creates proper state sequences

**Expected output:**

.. code-block:: text

   Testing Polynomial Trajectory Generation...

   1. Quintic polynomial (lateral trajectory)
      Lane change: d=0 to d=3.5m in T=4s
      Coefficients: [ 0.       0.       0.       0.546875 -0.205...  0.0205...]
      At t=0: p=0.000, v=0.000, a=0.000
      At t=T: p=3.500, v=-0.000, a=0.000
      Boundary conditions: ✓

   2. Quartic polynomial (longitudinal trajectory)
      Speed up: v=20 to v=25 m/s in T=3s
      Coefficients: [ 0.      20.       0.       0.5556  -0.0926]
      At t=0: s=0.000, v=20.000, a=0.000
      At t=T: s=67.500, v=25.000, a=0.000
      Boundary conditions: ✓

   3. Full trajectory generation
      Generated 9 states:
        t=0.0: s=0.00m, d=0.00m, v=20.00m/s
        t=0.5: s=10.06m, d=0.06m, v=20.37m/s
        t=1.0: s=20.46m, d=0.36m, v=21.30m/s
        ...
        t=4.0: s=91.85m, d=3.50m, v=22.96m/s

   4. Candidate trajectory generation
      Generated 125 candidate trajectories

   ==================================================
   All tests complete.

.. code-block:: bash

   # Test cost.py implementation
   python3 cost.py

**cost.py** tests cost function and feasibility checking:

- Creates a test trajectory using your polynomial implementation
- Tests ``check_feasibility()`` with normal and strict limits
- Tests ``compute_total_cost()`` with matching and different targets
- Tests ``select_best_trajectory()`` from candidate set
- Verifies correct handling when no feasible trajectory exists

**Expected output:**

.. code-block:: text

   Testing Cost Function and Feasibility...

   1. Creating test trajectory (lane change):
      Duration: 4.0s
      States: 41
      Final d: 3.50m
      Final v: 25.00m/s

   2. Testing feasibility checking:
      Default limits: ✓ Feasible
      Strict limits (v<20): ✗ Infeasible (expected)

   3. Testing cost computation:
      Cost (matching targets): 17.02
      Cost (different targets): 54.27
      Off-target cost should be higher: ✓

   4. Testing trajectory selection:
      Generated 125 candidates
      Best trajectory:
        Duration: 5.0s
        Final d: 3.00m
        Final v: 25.00m/s
        Cost: 8.77
      ✓ Selection working

   ==================================================
   All tests complete.

Running Simulation Scenarios
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

After all unit tests pass, run the full integrated simulation:

**Scenario 1: Empty Road (Lane Keeping)**

.. code-block:: bash

   # With visualization
   python3 simulator.py --scenario empty
   
   # Without visualization (text output)
   python3 simulator.py --scenario empty --no-viz --duration 20

**Demo:**


.. video:: /_static/final/frenet_empty.mp4
   :width: 640
   :height: 360

**Scenario 2: Follow (Vehicle Following)**

.. code-block:: bash

   # With visualization
   python3 simulator.py --scenario follow
   
   # Without visualization
   python3 simulator.py --scenario follow --no-viz --duration 30

**Demo:**


.. video:: /_static/final/frenet_follow.mp4
   :width: 640
   :height: 360

**Scenario 3: Overtake (Lane Changes)**

.. code-block:: bash

   # With visualization
   python3 simulator.py --scenario overtake
   
   # Without visualization
   python3 simulator.py --scenario overtake --no-viz --duration 40

**Demo:**


.. video:: /_static/final/frenet_overtake.mp4
   :width: 640
   :height: 360

Understanding the Visualization
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The visualization displays three panels:

**Main Panel (Top):**

- Green rectangle: Ego vehicle with path trail showing history
- Red rectangles: Traffic vehicles
- Blue line: Planned trajectory
- Blue dots: Trajectory waypoints
- Top-left text box: Time, speed, s position, d position
- Lower-left text box: Current behavior with color coding

  - LANE KEEP: Green
  - FOLLOW VEHICLE: Orange
  - LANE CHANGE LEFT: Blue
  - LANE CHANGE RIGHT: Purple

**Behavior Summary Panel (Bottom Left):**

Displays a log of behavior transitions during the simulation:

.. code-block:: text

   Time      Pos       Behavior
   -----------------------------------
     0.0s     0.0m  LANE CHANGE LEFT
     3.3s    68.0m  LANE CHANGE RIGHT
     6.6s   151.6m  LANE CHANGE LEFT
     ...

**Polynomial Coefficients Panel (Bottom Right):**

Shows the current trajectory's polynomial parameters:

.. code-block:: text

   Duration: T = 4.00s

   Longitudinal (s):
     Start: s=0.0m, ṡ=25.0m/s
     End:   s=100.0m, ṡ=31.0m/s
     Coeffs: [0.00, 25.00, 0.00, 0.75...]

   Lateral (d):
     Start: d=0.00m, ḋ=0.00m/s
     End:   d=3.50m, ḋ=0.00m/s
     Coeffs: [0.00, 0.00, 0.00, 1.09...]

   Cost: 45.2

Evaluation Checklist
~~~~~~~~~~~~~~~~~~~~~

Before submission, verify:

☐ ``test_frenet.py`` — 6/6 tests pass

☐ ``test_polynomial.py`` — 9/9 tests pass

☐ ``test_cost.py`` — 9/9 tests pass

☐ ``--scenario empty`` — Completes, maintains lane

☐ ``--scenario follow`` — Completes, follows lead vehicle

☐ ``--scenario overtake`` — Completes all lane changes

☐ No collisions in any scenario

☐ Smooth trajectories (no jerky motion)


---------------------------------------------------------
8. Configuration Parameters
---------------------------------------------------------

Vehicle Constraint Parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Defined in ``cost.py``:

.. code-block:: python

   @dataclass
   class FeasibilityLimits:
       max_velocity: float = 40.0       # m/s (~90 mph)
       max_acceleration: float = 4.0    # m/s²
       max_deceleration: float = -8.0   # m/s² (negative)
       max_lateral_accel: float = 3.0   # m/s²
       max_jerk: float = 10.0           # m/s³

Cost Function Weights
~~~~~~~~~~~~~~~~~~~~~~

Defined in ``cost.py``:

.. code-block:: python

   @dataclass
   class CostWeights:
       w_jerk: float = 1.0       # Penalize jerky motion
       w_time: float = 1.0       # Penalize long trajectories
       w_d: float = 1.0          # Penalize lateral deviation
       w_v: float = 1.0          # Penalize speed deviation
       w_accel: float = 1.0      # Penalize high acceleration

**Tuning tips:**

- Increase ``w_jerk`` for smoother trajectories
- Increase ``w_d`` to reach target lane faster
- Increase ``w_v`` to match target speed faster
- Decrease ``w_time`` if trajectories are too short

---------------------------------------------------------
9. Provided Package Structure
---------------------------------------------------------

The starter package includes all necessary files. RWA3 behavior tree files are already included.

.. code-block:: text

   final_project_starter/
   ├── frenet.py               # TODO: Implement Frenet transforms
   ├── polynomial.py           # TODO: Implement polynomial trajectories
   ├── cost.py                 # TODO: Implement cost functions
   ├── bt_framework.py         # Provided (behavior tree base classes)
   ├── bt_nodes.py             # Provided (behavior tree nodes)
   ├── behavior_tree.py        # Provided (behavioral planner)
   ├── simulator.py            # Provided (integrated simulator)
   ├── visualizer.py           # Provided (visualization)
   ├── test_frenet.py          # Unit tests for frenet.py
   ├── test_polynomial.py      # Unit tests for polynomial.py
   ├── test_cost.py            # Unit tests for cost.py
   ├── README.md               # Quick start guide
   └── requirements.txt        # numpy, matplotlib

Visualization Layout
~~~~~~~~~~~~~~~~~~~~~

The simulator provides a three-panel visualization:

**Main Panel (Top):** Shows the curved road with ego vehicle (green), traffic vehicles (red), planned trajectory (blue line), and current behavior with color-coded text.

**Behavior Summary (Bottom Left):** Logs each behavior transition with timestamp and position, showing the last 8 transitions.

**Polynomial Coefficients (Bottom Right):** Displays the current trajectory's start/end states and polynomial coefficients for both longitudinal (s) and lateral (d) motion.

.. code-block:: text

   ┌─────────────────────────────────────────────────────────────────┐
   │                       ROAD VIEW (Top Panel)                     │
   │  - Green vehicle: Ego with path trail                          │
   │  - Red vehicles: Traffic                                        │
   │  - Blue line/dots: Planned trajectory                          │
   │  - Behavior text with color coding (green/orange/blue/purple)  │
   ├────────────────────────────┬────────────────────────────────────┤
   │   BEHAVIOR SUMMARY         │   POLYNOMIAL COEFFICIENTS          │
   │   (Bottom Left)            │   (Bottom Right)                   │
   │                            │                                    │
   │   Time      Pos  Behavior  │   Duration: T = 4.00s              │
   │   0.0s    0.0m  LANE KEEP  │                                    │
   │   3.3s   68.0m  LANE CHG L │   Longitudinal (s):                │
   │   6.6s  151.6m  LANE CHG R │     Start: s=0m, ṡ=25m/s           │
   │   ...                      │     End: s=100m, ṡ=31m/s           │
   │                            │     Coeffs: [0.00, 25.00, ...]     │
   │                            │                                    │
   │                            │   Lateral (d):                     │
   │                            │     Start: d=0m, ḋ=0m/s            │
   │                            │     End: d=3.5m, ḋ=0m/s            │
   │                            │     Coeffs: [0.00, 0.00, ...]      │
   │                            │                                    │
   │                            │   Cost: 45.2                       │
   └────────────────────────────┴────────────────────────────────────┘

**Behavior Colors:**

- LANE KEEP: Green
- FOLLOW VEHICLE: Orange
- LANE CHANGE LEFT: Blue
- LANE CHANGE RIGHT: Purple

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
   ├── bt_framework.py               # Provided (include as-is)
   ├── bt_nodes.py                   # Provided (include as-is)
   ├── behavior_tree.py              # Provided (include as-is)
   ├── simulator.py                  # Provided (include as-is)
   ├── visualizer.py                 # Provided (include as-is)
   ├── results/
   │   ├── scenario_empty.png        # Screenshot of visualization
   │   ├── scenario_follow.png       # Screenshot of visualization
   │   ├── scenario_overtake.png     # Screenshot of visualization
   │   └── test_output.txt           # Unit test results
   └── report.pdf                    # Required (3-5 pages)

Generating test_output.txt
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   python3 test_frenet.py > results/test_output.txt 2>&1
   python3 test_polynomial.py >> results/test_output.txt 2>&1
   python3 test_cost.py >> results/test_output.txt 2>&1

Report Requirements
~~~~~~~~~~~~~~~~~~~~

**Length:** 3-5 pages maximum (excluding figures)

**Required Sections:**

1. **Frenet Transform:** Implementation approach, roundtrip accuracy, challenges

2. **Polynomial Trajectories:** Mathematical derivation, boundary condition verification

3. **Cost Function:** Cost components, weight tuning process

4. **Results:** Trajectory plots from each scenario, discussion of quality

5. **Discussion:** What worked, what was challenging, potential improvements

---------------------------------------------------------
11. Grading Rubric
---------------------------------------------------------

**Total: 60 points**

**Frenet Transform (15 pts):**

- ``cartesian_to_frenet`` (7 pts): Correct s, d computation
- ``frenet_to_cartesian`` (5 pts): Correct x, y recovery
- Roundtrip accuracy (3 pts): Error < 0.5 m

**Polynomial Trajectories (15 pts):**

- ``quintic_coefficients`` (6 pts): All 6 boundary conditions satisfied
- ``quartic_coefficients`` (4 pts): All 5 boundary conditions satisfied
- ``generate_trajectory`` (5 pts): Correct state evaluation

**Cost Function (15 pts):**

- ``compute_total_cost`` (6 pts): All components correct
- ``check_feasibility`` (5 pts): All constraints checked
- ``select_best_trajectory`` (4 pts): Correct selection

**Integration & Results (15 pts):**

- Scenarios pass (9 pts): 3 pts each for empty, follow, overtake
- Trajectory quality (3 pts): Smooth, comfortable motion
- Report (3 pts): Complete, well-written


---------------------------------------------------------
12. Quick Reference: Running Everything
---------------------------------------------------------

.. code-block:: bash

   # 1. Setup
   cd final_project_starter/
   pip install numpy matplotlib
   
   # 2. Run unit tests (implement functions first!)
   python3 test_frenet.py
   python3 test_polynomial.py
   python3 test_cost.py
   
   # 3. Run simulations with visualization
   python3 simulator.py --scenario empty
   python3 simulator.py --scenario follow
   python3 simulator.py --scenario overtake
   
   # 4. Run simulations without visualization (text output)
   python3 simulator.py --scenario overtake --no-viz --duration 30
   
   # 5. Generate test output for submission
   mkdir -p results
   python3 test_frenet.py > results/test_output.txt 2>&1
   python3 test_polynomial.py >> results/test_output.txt 2>&1
   python3 test_cost.py >> results/test_output.txt 2>&1
   

Behavior Summary Output
~~~~~~~~~~~~~~~~~~~~~~~~

When running with ``--no-viz``, a behavior summary is displayed at the end:

.. code-block:: text

   ============================================================
   BEHAVIOR SUMMARY
   ============================================================
       Time    Position  Behavior            
   --------  ----------  --------------------
       0.0s        0.0m  LANE CHANGE LEFT    
       3.3s       68.0m  LANE CHANGE RIGHT   
       6.6s      151.6m  LANE CHANGE LEFT    
       9.8s      232.3m  LANE CHANGE RIGHT   
      11.4s      275.8m  FOLLOW VEHICLE      
      13.8s      335.4m  LANE CHANGE RIGHT   
      14.8s      357.0m  LANE KEEP           
   ============================================================

---------------------------------------------------------
13. References
---------------------------------------------------------

**Primary References:**

- Werling, M., Ziegler, J., Kammel, S., & Thrun, S. (2010). *Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame.* IEEE ICRA.
- González, D., Pérez, J., Milanés, V., & Nashashibi, F. (2016). *A Review of Motion Planning Techniques for Automated Vehicles.* IEEE T-ITS.

**Additional Reading:**

- Paden, B., et al. (2016). *A Survey of Motion Planning and Control Techniques Adopted in Self-Driving Vehicles.* IEEE T-IV.