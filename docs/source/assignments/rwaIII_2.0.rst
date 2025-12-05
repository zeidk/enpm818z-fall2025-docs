==================================================================
Assignment 3
==================================================================

:Title: Behavioral Planning with Behavior Trees

:Course: ENPM818Z — On-Road Automated Vehicles
:Topic: L5 — Behavioral Planning
:Assigned: December 1, 2025
:Due: December 13, 2025
:Total Points: 40 pts (+10 bonus)
:Language: Python

.. admonition:: Resources
   :class: resources

   - 🔗 `Starter package <https://github.com/zeidk/enpm818Z-fall-2025/tree/main/rwa3_starter>`_
   
   - 📖 Lecture Materials: L5 (Behavioral Planning - Behavior Trees)

.. admonition:: Changelog

   **Version 2.0.0** (2025-12-05)

   - Refactored to use standalone simulator (no CARLA dependency)
   - CARLA integration now optional for bonus points
   - Added comprehensive unit tests

   **Version 1.0.0** (2025-11-30)

   - First version



.. note::

   **Project Structure:**
   
   This assignment focuses on **Behavioral Planning** using Behavior Trees (Part 1 of the Planning Stack). The subsequent **Final Project** will cover **Trajectory Planning** using Frenet coordinates and polynomial trajectories (Part 2 of the Planning Stack). Together, these implement a complete hierarchical planning system for autonomous driving.

   **Simulator Options:**
   
   - **Primary:** Standalone Python simulator (no external dependencies)
   - **Bonus (+10 pts):** CARLA integration for realistic 3D visualization

---------------------------------------------------------
1. Objective
---------------------------------------------------------

Implement a **behavioral planner** using Behavior Trees that decides high-level driving maneuvers for an autonomous vehicle.

**Learning Objectives:**

- Apply Behavior Tree concepts to implement a modular behavioral planner
- Design condition nodes that evaluate driving situations
- Design action nodes that set appropriate behavioral commands
- Test and validate decision-making behavior in simulation

---------------------------------------------------------
2. Background
---------------------------------------------------------

In the lecture, we discussed hierarchical architectures for autonomous driving where the **behavioral planner** sits between perception and trajectory planning. The behavioral planner answers the question: *"What should the vehicle do?"* — for example, keep lane, follow a vehicle, change lanes, or stop.

**Behavior Trees (BTs)** provide a modular, hierarchical structure for decision-making:

- **Modularity:** Each node encapsulates a specific behavior or condition
- **Reusability:** Nodes can be reused across different tree configurations
- **Debuggability:** Tree structure makes execution flow easy to trace
- **Extensibility:** New behaviors can be added without modifying existing nodes

**Tick Mechanism:**

The BT is executed ("ticked") every planning cycle. Each tick traverses the tree from the root, and each node returns one of three statuses:

.. list-table::
   :widths: 20 80
   :header-rows: 1
   :class: compact-table

   * - **Status**
     - **Meaning**
   * - SUCCESS
     - The node completed its task successfully
   * - FAILURE
     - The node failed to complete its task
   * - RUNNING
     - The node is still executing (will continue next tick)

**Node Types:**

.. list-table::
   :widths: 20 80
   :header-rows: 1
   :class: compact-table

   * - **Node Type**
     - **Behavior**
   * - Sequence (→)
     - Executes children left-to-right. Returns SUCCESS if ALL children succeed. Returns FAILURE immediately if any child fails.
   * - Selector (?)
     - Executes children left-to-right. Returns SUCCESS immediately if any child succeeds. Returns FAILURE if ALL children fail.
   * - Condition
     - Checks a condition (e.g., "Is lane clear?"). Returns SUCCESS or FAILURE.
   * - Action
     - Performs an action (e.g., "Set target lane"). Returns SUCCESS after setting the command.

---------------------------------------------------------
3. System Architecture
---------------------------------------------------------

What You Receive vs. Implement
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 45 15 15
   :header-rows: 1
   :class: compact-table

   * - **File**
     - **Provided**
     - **You Implement**
   * - ``bt_framework.py`` — BT base classes, dataclasses
     - ✓
     - 
   * - ``bt_nodes.py`` — Condition and action nodes
     - 
     - ✓
   * - ``behavior_tree.py`` — Tree assembly (``_build_tree()``)
     - 
     - ✓
   * - ``test_behavior_tree.py`` — Unit tests (16 tests)
     - ✓
     - 
   * - ``simulator.py`` — Highway driving simulator
     - ✓
     - 
   * - ``visualizer.py`` — Matplotlib visualization
     - ✓
     - 
   * - ``requirements.txt`` — Python dependencies
     - ✓
     - 
   * - ``README.md`` — Setup instructions
     - ✓
     - 

Data Flow
~~~~~~~~~

.. only:: html

   .. figure:: /_static/final/data_flow_light.png
      :alt: Data Flow
      :align: center
      :width: 90%
      :class: only-light

   .. figure:: /_static/final/data_flow_dark.png
      :alt: Data Flow
      :align: center
      :width: 90%
      :class: only-dark


**Key Interfaces:**

The ``EnvironmentState`` dataclass provides perception data:

.. code-block:: python

   @dataclass
   class EnvironmentState:
       ego_speed: float           # Current speed (m/s)
       ego_d: float               # Lateral offset from centerline (m)
       speed_limit: float         # Speed limit (m/s)
       left_lane_exists: bool     # Is there a lane to the left?
       right_lane_exists: bool    # Is there a lane to the right?
       left_lane_clear: bool      # Is left lane safe to enter?
       right_lane_clear: bool     # Is right lane safe to enter?
       vehicle_ahead: bool        # Is there a vehicle ahead?
       vehicle_ahead_distance: float  # Distance to vehicle ahead (m)
       vehicle_ahead_speed: float     # Speed of vehicle ahead (m/s)

Your planner outputs a ``BehaviorCommand``:

.. code-block:: python

   @dataclass
   class BehaviorCommand:
       behavior: BehaviorType     # LANE_KEEP, FOLLOW_VEHICLE, LANE_CHANGE_LEFT, etc.
       target_d: float            # Target lateral offset (m)
       target_speed: float        # Target speed (m/s)
       T: float                   # Planning horizon (s)

---------------------------------------------------------
4. Configuration Parameters
---------------------------------------------------------

The following constants are defined at the **top of** ``bt_nodes.py`` **(lines 28-31)**. Use these in your implementation:

.. code-block:: python

   # bt_nodes.py (lines 28-31)
   SPEED_LIMIT = 31.0       # m/s (~70 mph)
   LANE_WIDTH = 3.5         # meters
   FOLLOW_DISTANCE = 50.0   # meters - distance to trigger following
   SLOW_THRESHOLD = 5.0     # m/s - speed difference to consider vehicle "slow"

.. note::

   There is no separate configuration file. These parameters are defined directly in ``bt_nodes.py`` and are already provided for you. Do not modify these values.

---------------------------------------------------------
5. Implementation Tasks
---------------------------------------------------------

Step 1: Implement Condition Nodes (bt_nodes.py)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**IsVehicleAhead** — Check if a blocking vehicle is ahead:

.. code-block:: python

   def update(self) -> Status:
       """
       Return SUCCESS if:
       - vehicle_ahead is True AND
       - vehicle_ahead_distance < FOLLOW_DISTANCE AND
       - vehicle_ahead_speed < speed_limit - 1.0 (blocking our cruise)
       
       Return FAILURE otherwise.
       """

**IsVehicleSlow** — Check if vehicle ahead is slow enough to overtake:

.. code-block:: python

   def update(self) -> Status:
       """
       Return SUCCESS if:
       - vehicle_ahead is True AND
       - (speed_limit - vehicle_ahead_speed) > SLOW_THRESHOLD
       
       Return FAILURE otherwise.
       """

**IsLaneChangeSafe** — Check if lane change is safe (prefer left):

.. code-block:: python

   def update(self) -> Status:
       """
       Return SUCCESS if left lane exists and clear (store 'left' on blackboard)
       Else return SUCCESS if right lane exists and clear (store 'right' on blackboard)
       Return FAILURE if no lane change is safe.
       """

Step 2: Implement Action Nodes (bt_nodes.py)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**SetLaneKeepCommand** — Default cruising behavior:

.. code-block:: python

   def update(self) -> Status:
       """
       Set BehaviorCommand with:
       - behavior = LANE_KEEP
       - target_d = 0.0 (center of lane)
       - target_speed = speed_limit
       - T = 3.0
       Return SUCCESS.
       """

**SetFollowCommand** — Follow the vehicle ahead:

.. code-block:: python

   def update(self) -> Status:
       """
       Set BehaviorCommand with:
       - behavior = FOLLOW_VEHICLE
       - target_d = 0.0 (stay in lane)
       - target_speed = vehicle_ahead_speed - 1.0 (maintain gap)
       - T = 5.0
       Return SUCCESS.
       """

**SetLaneChangeCommand** — Change to target lane:

.. code-block:: python

   def update(self) -> Status:
       """
       Read target_lane from blackboard ('left' or 'right')
       Set BehaviorCommand with:
       - behavior = LANE_CHANGE_LEFT or LANE_CHANGE_RIGHT
       - target_d = +LANE_WIDTH (left) or -LANE_WIDTH (right)
       - target_speed = speed_limit
       - T = 4.0
       Return SUCCESS.
       """

Step 3: Build the Behavior Tree (behavior_tree.py)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Implement ``_build_tree()`` to create this structure:

.. code-block:: text

   Root [Selector]
   ├── Lane Change [Sequence]
   │   ├── IsVehicleAhead
   │   ├── IsVehicleSlow
   │   ├── IsLaneChangeSafe
   │   └── SetLaneChangeCommand
   ├── Follow Vehicle [Sequence]
   │   ├── IsVehicleAhead
   │   └── SetFollowCommand
   └── Lane Keep [Sequence]
       └── SetLaneKeepCommand

**Priority Logic:**

1. **Lane Change:** If there's a slow vehicle ahead AND a safe lane → change lanes
2. **Follow:** If there's a vehicle ahead but can't change lanes → follow it
3. **Lane Keep:** Default behavior when road is clear

---------------------------------------------------------
6. Testing Your Implementation
---------------------------------------------------------

There are two ways to test your implementation:

1. **Unit Tests (test_behavior_tree.py)** — Automated tests that verify each node
2. **Simulator (simulator.py)** — Visual simulation to see your behavior tree in action

Unit Tests: test_behavior_tree.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This file contains **16 automated tests** that verify your implementation without requiring visualization. The tests create ``EnvironmentState`` objects with specific values and check that your nodes return the correct ``Status`` and ``BehaviorCommand``.

**What the tests cover:**

.. list-table::
   :widths: 35 65
   :header-rows: 1
   :class: compact-table

   * - **Test Category**
     - **Tests Included**
   * - **IsVehicleAhead (3 tests)**
     - | No vehicle :math:`\rightarrow` FAILURE
       | Vehicle far (70m) :math:`\rightarrow` FAILURE
       | Vehicle close and slow :math:`\rightarrow` SUCCESS
   * - **IsVehicleSlow (3 tests)**
     - | No vehicle :math:`\rightarrow` FAILURE
       | Fast vehicle (29 m/s) :math:`\rightarrow` FAILURE
       | Slow vehicle (22 m/s) :math:`\rightarrow` SUCCESS
   * - **IsLaneChangeSafe (3 tests)**
     - | Both lanes clear :math:`\rightarrow` SUCCESS (left)
       | Only right clear :math:`\rightarrow` SUCCESS (right)
       | No lanes clear :math:`\rightarrow` FAILURE
   * - **SetLaneKeepCommand (1 test)**
     - Verifies behavior=LANE_KEEP, target_d=0, target_speed=31
   * - **SetFollowCommand (1 test)**
     - Verifies behavior=FOLLOW_VEHICLE, target_speed=vehicle_speed-2
   * - **SetLaneChangeCommand (2 tests)**
     - | Left: behavior=LANE_CHANGE_LEFT, target_d=3.5
       | Right: behavior=LANE_CHANGE_RIGHT, target_d=-3.5
   * - **Integration (3 tests)**
     - | Empty road :math:`\rightarrow` lane_keep
       | Slow vehicle + clear lane :math:`\rightarrow` lane_change
       | Blocked lanes :math:`\rightarrow` follow

**Run all unit tests:**

.. code-block:: bash

   python3 test_behavior_tree.py

**Expected output when all tests pass:**

.. code-block:: text

   ============================================================
   BEHAVIOR TREE UNIT TESTS
   ============================================================

   --- Condition Node Tests ---

   IsVehicleAhead:
     [PASS] No vehicle ahead -> FAILURE
     [PASS] Vehicle far (70m) -> FAILURE
     [PASS] Vehicle close and slow -> SUCCESS

   IsVehicleSlow:
     [PASS] No vehicle -> FAILURE
     [PASS] Fast vehicle (29 m/s) -> FAILURE
     [PASS] Slow vehicle (22 m/s) -> SUCCESS

   IsLaneChangeSafe:
     [PASS] Both clear -> SUCCESS, target=left
     [PASS] Only right clear -> SUCCESS, target=right
     [PASS] No lanes clear -> FAILURE

   --- Action Node Tests ---

   SetLaneKeepCommand:
     [PASS] behavior=lane_keep, d=0.0, v=31.0

   SetFollowCommand:
     [PASS] behavior=follow_vehicle, v=21.0 (expected ~21.0)

   SetLaneChangeCommand (left):
     [PASS] behavior=lane_change_left, d=3.5

   SetLaneChangeCommand (right):
     [PASS] behavior=lane_change_right, d=-3.5

   --- Behavior Planner Integration Tests ---

   Empty road scenario:
     [PASS] Empty road -> lane_keep

   Slow vehicle, left clear:
     [PASS] Slow vehicle, can pass -> lane_change_left

   Vehicle ahead, lanes blocked:
     [PASS] Can't pass -> follow_vehicle

   ============================================================
   RESULTS: 16 passed, 0 failed
   ============================================================

   ✅ All tests passed! Your implementation is correct.

**Save test output for submission:**

.. code-block:: bash

   python3 test_behavior_tree.py > results/test_output.txt 2>&1

Simulator: simulator.py
~~~~~~~~~~~~~~~~~~~~~~~

The simulator provides a **visual environment** to see your behavior tree making decisions in real-time. It simulates a 3-lane highway with an ego vehicle (controlled by your behavior tree) and traffic vehicles.

**What the simulator does:**

1. **Initializes** the highway environment with ego vehicle at position (0, 0)
2. **Sets up traffic** based on the selected scenario
3. **Runs a loop** where each iteration:
   
   - Calls ``get_environment_state()`` to get perception data
   - Passes the state to your ``BehaviorPlanner.get_command()``
   - Applies the returned ``BehaviorCommand`` to update vehicle motion
   - Updates visualization (if enabled)

4. **Displays** the current behavior decision (LANE KEEP, FOLLOW VEHICLE, LANE CHANGE LEFT, etc.)

**Simulator command-line options:**

.. list-table::
   :widths: 25 75
   :header-rows: 1
   :class: compact-table

   * - **Option**
     - **Description**
   * - ``--scenario``
     - Choose scenario: ``empty``, ``follow``, or ``overtake``
   * - ``--duration``
     - Simulation length in seconds (default: 30)
   * - ``--no-viz``
     - Disable visualization, print text output only

Test Scenario: Empty Road
~~~~~~~~~~~~~~~~~~~~~~~~~~

**Description:** No traffic vehicles. The ego vehicle should cruise at the speed limit using LANE KEEP behavior.

**Expected behavior:** LANE KEEP throughout the simulation.

.. code-block:: bash

   # With visualization
   python3 simulator.py --scenario empty
   
   # Text-only mode
   python3 simulator.py --scenario empty --no-viz --duration 20

**Demo:**


.. video:: /_static/final/empty.mp4
   :width: 640
   :height: 360

Test Scenario: Follow Vehicle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Description:** All three red vehicles drive at the same speed (22 m/s). The adjacent vehicles start behind the ego and gradually catch up, keeping the lanes blocked throughout the simulation. The ego can only follow the lead vehicle.

**Expected behavior:** FOLLOW VEHICLE throughout the simulation (no lane changes possible).

.. code-block:: bash

   # With visualization
   python3 simulator.py --scenario follow
   
   # Text-only mode
   python3 simulator.py --scenario follow --no-viz --duration 30

**Demo:**

.. video:: /_static/final/follow.mp4
   :width: 640
   :height: 360

Test Scenario: Overtake
~~~~~~~~~~~~~~~~~~~~~~~~

**Description:** A slow vehicle is ahead in the center lane with the left lane clear. Further down the road, another slow vehicle is in the left lane. The ego vehicle should:

1. Change left to pass the first slow vehicle
2. Continue in the left lane
3. Change right when approaching the slow vehicle in the left lane

**Expected behavior:** LANE CHANGE LEFT → LANE KEEP → LANE CHANGE RIGHT

.. code-block:: bash

   # With visualization
   python3 simulator.py --scenario overtake
   
   # Text-only mode  
   python3 simulator.py --scenario overtake --no-viz --duration 40

**Demo:**

.. video:: /_static/final/overtake.mp4
   :width: 640
   :height: 360

---------------------------------------------------------
7. Provided Package Structure
---------------------------------------------------------

.. code-block:: text

   rwa3_starter/
   ├── bt_framework.py           # BT base classes, EnvironmentState, BehaviorCommand
   ├── bt_nodes.py               # TODO: Implement condition/action nodes
   ├── behavior_tree.py          # TODO: Implement _build_tree()
   ├── test_behavior_tree.py     # Unit tests (16 tests)
   ├── simulator.py              # Highway simulator with 3 scenarios
   ├── visualizer.py             # Matplotlib real-time visualization
   ├── requirements.txt          # numpy, matplotlib
   └── README.md                 # Setup and usage instructions

**Files you implement (2 files):**

- ``bt_nodes.py`` — Implement 3 condition nodes and 3 action nodes (look for ``# TODO`` comments)
- ``behavior_tree.py`` — Implement ``_build_tree()`` method (look for ``# TODO`` comments)

**Files you should NOT modify (6 files):**

- ``bt_framework.py`` — Contains ``Status``, ``BehaviorType``, ``EnvironmentState``, ``BehaviorCommand``, ``Node``, ``Sequence``, ``Selector``, ``ConditionNode``, ``ActionNode``, ``Blackboard``
- ``test_behavior_tree.py`` — Contains 16 unit tests that verify your implementation
- ``simulator.py`` — Contains ``Simulator`` class with 3 scenarios (empty, follow, overtake)
- ``visualizer.py`` — Contains ``SimulationVisualizer`` class for matplotlib animation
- ``requirements.txt`` — Dependencies (numpy, matplotlib)
- ``README.md`` — Instructions

---------------------------------------------------------
8. Submission Requirements
---------------------------------------------------------

Submit a **single ZIP file** named ``groupX_rwa3.zip`` containing:

.. code-block:: text

   groupX_rwa3/
   ├── bt_nodes.py                   # Your implementation
   ├── behavior_tree.py              # Your implementation
   ├── results/
   │   └── test_output.txt           # Output from: python3 test_behavior_tree.py
   └── README.md                     # Team info, instructions, notes

**README Requirements:**

1. **Team Members:** Names and UIDs
2. **Build/Run Instructions:** How to run your code
3. **Implementation Notes:** Key design decisions, challenges faced
4. **Test Results:** Summary of passing/failing tests

**Submission Checklist:**

- ☐ All condition nodes implemented and tested
- ☐ All action nodes implemented and tested
- ☐ ``_build_tree()`` creates correct tree structure
- ☐ All 16 unit tests pass
- ☐ Test output saved to ``results/test_output.txt``
- ☐ Code is well-commented
- ☐ ZIP file follows naming convention

---------------------------------------------------------
9. Grading Rubric
---------------------------------------------------------

**Total: 40 points**

**Condition Nodes (12 pts):**

- **IsVehicleAhead (4 pts):** Correctly identifies blocking vehicle
- **IsVehicleSlow (4 pts):** Correctly evaluates speed difference threshold
- **IsLaneChangeSafe (4 pts):** Correctly checks lane availability (prefers left)

**Action Nodes (15 pts):**

- **SetLaneKeepCommand (4 pts):** Sets correct maneuver, lane, and speed
- **SetFollowCommand (5 pts):** Correctly matches lead vehicle speed with gap
- **SetLaneChangeCommand (6 pts):** Correct direction and target_d

**Tree Assembly (8 pts):**

- **Correct structure (5 pts):** All branches in correct priority order
- **Integration (3 pts):** Tree ticks correctly, all scenarios work

**Documentation (5 pts):**

- Complete README, well-commented code, test output saved

---------------------------------------------------------
10. Bonus: CARLA Integration (+10 pts)
---------------------------------------------------------

For additional credit, integrate your behavioral planner with the CARLA simulator.

**Requirements:**

1. Install CARLA 0.9.13 or later
2. Implement ``carla_interface.py`` to connect to CARLA
3. Map CARLA perception data to ``EnvironmentState``
4. Run all three scenarios in CARLA

**Bonus Grading:**

- **+5 pts:** CARLA connection works, vehicle spawns and moves
- **+3 pts:** Perception correctly mapped to EnvironmentState
- **+2 pts:** All scenarios complete successfully in CARLA

**Submission:** Include a ``carla/`` folder with your CARLA integration code and a video recording of each scenario (MP4, max 30 seconds each).

---------------------------------------------------------
11. References
---------------------------------------------------------

**Primary References:**

- Colledanchise, M. & Ögren, P. (2018). *Behavior Trees in Robotics and AI: An Introduction.* CRC Press.
- Behavior Trees in Games: https://www.behaviortree.dev/

**Additional Reading:**

- Paden, B., et al. (2016). *A Survey of Motion Planning and Control Techniques Adopted in Self-Driving Vehicles.* IEEE Transactions on Intelligent Vehicles.