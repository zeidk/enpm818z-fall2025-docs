==================================================================
Assignment 3
==================================================================

:Title: Behavioral Planning with Behavior Trees

:Course: ENPM818Z — On-Road Automated Vehicles
:Topic: L5 — Behavioral Planning
:Assigned: December 1, 2025
:Due: December 13, 2025
:Total Points: 40 pts (+20 bonus)
:Language: Python

.. admonition:: Resources
   :class: resources

   - 🔗 `Starter package <https://github.com/zeidk/enpm818Z-fall-2025/tree/main/rwa3_starter>`_
   
   - 📖 Lecture Materials: L5 (Behavioral Planning - Behavior Trees)

.. admonition:: Changelog

   **Version 2.1.0** (2025-12-10)

   - Added comprehensive :ref:`CARLA integration guide <carla-integration>` for bonus points (+20 pts)
   - Added Town04 additional maps installation instructions for Docker and native
   - CARLA scripts now included in starter package
   - Students implement only ``get_environment_state()`` method

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
   - **Bonus (+20 pts):** CARLA integration for realistic 3D visualization

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

.. _carla-integration:

---------------------------------------------------------
10. Bonus: CARLA Integration (+20 pts)
---------------------------------------------------------

For additional credit, run your behavioral planner in the CARLA 3D driving simulator.

.. note::

   CARLA integration is **optional**. You can earn full marks (40 pts) without it.

Overview
~~~~~~~~

The CARLA integration uses the **same behavior tree** you implemented for the standalone simulator. The only difference is how perception data is obtained—instead of a simple 2D simulation, you extract it from CARLA's 3D world.

.. warning::

   **Do not move or rename the** ``carla/`` **folder!**
   
   The starter package includes a ``carla/`` folder inside ``rwa3_starter/``. Keep this folder structure intact:
   
   .. code-block:: text
   
      rwa3_starter/
      ├── bt_framework.py
      ├── bt_nodes.py
      ├── behavior_tree.py
      ├── simulator.py
      ├── visualizer.py
      ├── test_behavior_tree.py
      └── carla/                    # DO NOT MOVE THIS FOLDER
          ├── carla_interface.py    # You implement get_environment_state()
          ├── carla_controller.py
          ├── carla_simulator.py
          ├── reset_carla.py
          ├── verify_setup.py
          └── README.md
   
   The scripts import from the parent directory (``..``) and will break if moved.

CARLA Scripts Overview
~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 25 55 20
   :header-rows: 1
   :class: compact-table

   * - **File**
     - **Description**
     - **You Modify?**
   * - ``carla_interface.py``
     - Bridge between your behavior tree and CARLA. Handles connection, vehicle spawning, perception extraction, and control. **You implement** ``get_environment_state()``.
     - **YES**
   * - ``carla_controller.py``
     - Low-level vehicle controller using CARLA's autopilot and traffic manager. Handles lane keeping, lane changes, and speed control.
     - No
   * - ``carla_simulator.py``
     - Main entry point. Runs the simulation loop: gets perception → calls your planner → applies commands. Includes pygame visualization and HUD.
     - No
   * - ``reset_carla.py``
     - Utility to reset CARLA to a clean state. Run this if CARLA freezes or behaves unexpectedly (disables synchronous mode, destroys vehicles).
     - No
   * - ``verify_setup.py``
     - Verification script to check your CARLA installation. Tests connection, Town04 availability, vehicle spawning, and synchronous mode.
     - No

Prerequisites
~~~~~~~~~~~~~

Before starting, ensure you have:

1. **CARLA Server** running (Docker or native installation)
2. **Town04 Map** installed (highway map required for lane-change scenarios)
3. **Python CARLA package** installed (``pip install carla``)


Installing Additional Maps (Town04)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Town04 is a **highway map** required for lane-change scenarios. It is not included in the base CARLA installation.

.. warning::

   Without Town04, the overtake scenario will not work correctly. Run ``verify_setup.py`` to check if Town04 is available.

.. note::

   In the instructions below, replace ``0.9.16`` with your CARLA version if different. The additional maps archive (e.g., `AdditionalMaps_0.9.16.tar.gz`) must match your CARLA version (e.g., 0.9.16).

**For Docker Installation:**

.. code-block:: bash

   # Step 1: Download additional maps on your HOST machine
   cd ~/Downloads
   wget https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/AdditionalMaps_0.9.16.tar.gz

   # Step 2: Copy the archive into the running container
   docker cp AdditionalMaps_0.9.16.tar.gz carla-server:/home/carla/

   # Step 3: Enter the container as root and install
   docker exec -it --user root carla-server bash
   
   # Inside container:
   cd /home/carla
   tar -xzf AdditionalMaps_0.9.16.tar.gz
   ./ImportAssets.sh
   exit

   # Step 4: Restart container to apply changes
   docker restart carla-server

**For Native Installation:**

.. code-block:: bash

   # Step 1: Navigate to your CARLA installation directory
   cd /path/to/CARLA_0.9.16
   
   # Step 2: Download additional maps
   wget https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/AdditionalMaps_0.9.16.tar.gz
   
   # Step 3: Extract and import
   tar -xzf AdditionalMaps_0.9.16.tar.gz
   ./ImportAssets.sh
   
   # Step 4: Restart CARLA
   # Kill any running instance and start again
   ./CarlaUE4.sh <options>



Step 1: Verify Your Setup
~~~~~~~~~~~~~~~~~~~~~~~~~

Before implementing anything, verify your CARLA installation works correctly.

**Start CARLA server:**

.. code-block:: bash

   # Docker
   docker start -ai carla-server
   
   # Native
   ./CarlaUE4.sh <options>

**Run verification script:**

.. code-block:: bash

   cd rwa3_starter/carla
   python verify_setup.py

**Expected output (all checks should pass):**

.. code-block:: text

   ============================================================
   CARLA Setup Verification
   ============================================================
   
   [1/5] Testing connection...
     ✓ Connected to CARLA 0.9.16
   
   [2/5] Checking available maps...
     Found 12 maps: Town01, Town02, Town03, Town04, Town05...
     ✓ Town04 (highway map) is available
   
   [3/5] Loading Town04...
     ✓ Town04 loaded successfully
   
   [4/5] Testing vehicle spawn...
     ✓ Vehicle spawned at (5, -170)
     ✓ Adjacent lanes available: left, right
   
   [5/5] Testing synchronous mode...
     ✓ Synchronous mode works correctly
   
   ============================================================
   ✓ ALL CHECKS PASSED!
   ============================================================
   
   Your CARLA setup is ready. You can now run:
     python carla_simulator.py --scenario empty

**If Town04 is missing:**

.. code-block:: text

   [2/5] Checking available maps...
     Found 5 maps: Town01, Town02, Town03, Town05, Town10HD...
     ✗ WARNING: Town04 not found!
       Please install additional maps (see setup instructions)

Install the additional maps following the instructions in the `Installing Additional Maps (Town04)`_ section above.

Step 2: Implement ``get_environment_state()``
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Open ``carla/carla_interface.py`` and find the ``get_environment_state()`` method (around line 679). This is the **only method you need to implement**.

**Method signature:**

.. code-block:: python

   def get_environment_state(self) -> EnvironmentState:
       """
       Extract perception data from CARLA and return EnvironmentState.
       
       Returns:
           EnvironmentState with all perception fields populated
       """

**Fields to populate:**

.. list-table::
   :widths: 25 75
   :header-rows: 1
   :class: compact-table

   * - **Field**
     - **How to Compute**
   * - ``ego_speed``
     - Get velocity from ``self.ego_vehicle.get_velocity()``, compute magnitude
   * - ``ego_d``
     - Use provided helper: ``self._calculate_lateral_offset(transform, waypoint)``
   * - ``speed_limit``
     - Use ``self.config.speed_limit``
   * - ``left_lane_exists``
     - Check ``waypoint.get_left_lane()`` is not None and is a driving lane
   * - ``right_lane_exists``
     - Check ``waypoint.get_right_lane()`` is not None and is a driving lane
   * - ``left_lane_clear``
     - No traffic vehicle in left lane within ``lane_change_gap``
   * - ``right_lane_clear``
     - No traffic vehicle in right lane within ``lane_change_gap``
   * - ``vehicle_ahead``
     - True if traffic vehicle in same lane ahead within ``detection_range``
   * - ``vehicle_ahead_distance``
     - Distance to closest vehicle ahead in same lane
   * - ``vehicle_ahead_speed``
     - Speed of vehicle ahead

**Helper methods available:**

.. code-block:: python

   # Calculate lateral offset (which lane ego is in)
   ego_d = self._calculate_lateral_offset(ego_transform, ego_waypoint)
   
   # Calculate relative position of another vehicle
   # Returns: (longitudinal_offset, lateral_offset)
   # Positive longitudinal = ahead, Positive lateral = left
   rel_x, rel_d = self._calculate_relative_position(ego_transform, traffic_transform)

**Configuration parameters:**

.. code-block:: python

   self.config.lane_width = 3.5        # meters
   self.config.detection_range = 100.0  # meters
   self.config.lane_change_gap = 25.0   # meters
   self.config.speed_limit = 31.0       # m/s

**Lane boundaries for classification:**

- Same lane: ``|rel_d| < lane_width / 2``
- Left lane: ``lane_width / 2 < rel_d < 1.5 * lane_width``
- Right lane: ``-1.5 * lane_width < rel_d < -lane_width / 2``

Step 3: Run Scenarios
~~~~~~~~~~~~~~~~~~~~~

**Start CARLA server** (keep running in separate terminal):

.. code-block:: bash

   ./CarlaUE4.sh <options>

**Reset CARLA** (recommended before each run):

.. code-block:: bash

   cd rwa3_starter/carla
   python3 reset_carla.py

**Run scenarios:**

.. code-block:: bash

   # Empty road - tests lane keeping
   python3 carla_simulator.py --scenario empty --duration 60
   
   # Follow vehicle - tests vehicle following
   python3 carla_simulator.py --scenario follow --duration 60
   
   # Overtake - tests lane change decisions
   python3 carla_simulator.py --scenario overtake --duration 90

CARLA Scenario: Empty Road
~~~~~~~~~~~~~~~~~~~~~~~~~~

**Description:** No traffic vehicles. Ego vehicle should cruise at speed limit.

**Expected behavior:** ``LANE_KEEP`` throughout, stable driving through curves.

.. code-block:: bash

   python3 carla_simulator.py --scenario empty --duration 60

**Demo:**

.. video:: /_static/rwa3/carla_empty.mp4
   :width: 640
   :height: 360

CARLA Scenario: Follow Vehicle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Description:** Lead vehicle ahead with blockers in adjacent lanes preventing lane changes.

**Expected behavior:** ``FOLLOW_VEHICLE`` throughout, matching lead vehicle speed.

.. note::

   Behaviors may alternate between ``FOLLOW_VEHICLE`` and ``LANE_KEEP``: This is fine.


.. code-block:: bash

   python3 carla_simulator.py --scenario follow --duration 60



**Demo:**

.. video:: /_static/rwa3/carla_follow.mp4
   :width: 640
   :height: 360

CARLA Scenario: Overtake
~~~~~~~~~~~~~~~~~~~~~~~~

**Description:** Slow vehicle in center lane, left lane clear initially, another slow vehicle further ahead in left lane.

**Expected behavior:** ``LANE_CHANGE_LEFT``, ``LANE_KEEP``, ``LANE_CHANGE_RIGHT``, ``FOLLOW_VEHICLE``

.. code-block:: bash

   python3 carla_simulator.py --scenario overtake --duration 90

**Demo:**

.. video:: /_static/rwa3/carla_overtake.mp4
   :width: 640
   :height: 360

Troubleshooting
~~~~~~~~~~~~~~~

.. list-table::
   :widths: 30 70
   :header-rows: 1
   :class: compact-table

   * - **Issue**
     - **Solution**
   * - CARLA freezes / no response
     - Run ``python reset_carla.py`` to disable synchronous mode
   * - "Town04 not available"
     - Install additional maps (see ROS 2 Jazzy Setup Guide)
   * - Connection refused
     - Ensure CARLA server is running on port 2000
   * - Vehicle spawns then disappears
     - Map may not have multi-lane roads; ensure Town04 is loaded
   * - Pygame window doesn't appear
     - Check pygame is installed: ``pip install pygame``
   * - Import errors
     - Don't move the ``carla/`` folder; run from inside it

Bonus Grading
~~~~~~~~~~~~~

**Total: +20 points**

.. list-table::
   :widths: 60 20
   :header-rows: 1
   :class: compact-table

   * - **Criterion**
     - **Points**
   * - CARLA connection works, vehicle spawns and drives
     - +5
   * - ``get_environment_state()`` correctly extracts ego speed and lane info
     - +5
   * - ``get_environment_state()`` correctly detects traffic vehicles
     - +5
   * - All three scenarios complete successfully with correct behaviors
     - +5

.. note::

   Only include ``carla_interface.py`` in your submission (other CARLA scripts are provided and unchanged). Please include video recordings for each scenario and clearly show the behavior transitions from the terminal (like in the demos I provided above).

---------------------------------------------------------
11. References
---------------------------------------------------------

**Primary References:**

- Colledanchise, M. & Ögren, P. (2018). *Behavior Trees in Robotics and AI: An Introduction.* CRC Press.
- Behavior Trees in Games: https://www.behaviortree.dev/

**Additional Reading:**

- Paden, B., et al. (2016). *A Survey of Motion Planning and Control Techniques Adopted in Self-Driving Vehicles.* IEEE Transactions on Intelligent Vehicles.