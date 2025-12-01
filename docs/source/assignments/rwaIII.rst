==================================================================
Assignment 3
==================================================================

:Title: Behavioral Planning with Behavior Trees

:Course: ENPM818Z — On-Road Automated Vehicles
:Topic: L5 — Behavioral Planning
:Assigned: December 1, 2025
:Due: December 8, 2025 (1 week)
:Total Points: 40 pts
:Language: Python (CARLA)

.. admonition:: Resources
   :class: resources

   - 🔗 `Starter package <https://github.com/zeidk/enpm818Z-fall-2025/tree/main/rwa3_final_starter>`_
   
   - 📖 Lecture Materials: L5 (Behavioral Planning - Behavior Trees)
   
   - 📖 CARLA Documentation: https://carla.readthedocs.io/

.. admonition:: Changelog

   **Version 1.0.0** (2025-11-30)

   - First version



.. note::

   **Project Structure:**
   
   This assignment focuses on **Behavioral Planning** using Behavior Trees (Part 1 of the Planning Stack). The subsequent **Final Project** will cover **Trajectory Planning** using Frenet coordinates and polynomial trajectories (Part 2 of the Planning Stack). Together, these implement a complete hierarchical planning system for autonomous driving.

---------------------------------------------------------
1. Objective
---------------------------------------------------------

Implement a **behavioral planner** using Behavior Trees that decides high-level driving maneuvers for an autonomous vehicle in the CARLA simulator. Your implementation will integrate with provided perception, trajectory planning, and control modules to create a complete autonomous driving system.

**Learning Objectives:**

- Apply Behavior Tree concepts to implement a modular behavioral planner
- Design condition nodes that evaluate driving situations
- Design action nodes that set appropriate behavioral commands
- Handle multi-tick actions using the RUNNING state
- Test and validate decision-making behavior in simulation

---------------------------------------------------------
2. Background
---------------------------------------------------------

In the lecture, we discussed hierarchical architectures for autonomous driving where the **behavioral planner** sits between perception and trajectory planning. The behavioral planner answers the question: *"What should the vehicle do?"* — for example, keep lane, follow a vehicle, change lanes, or stop.

**Behavior Trees (BTs)** provide a modular, hierarchical structure for decision-making that offers several advantages over finite state machines:

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
     - Executes children left-to-right. Returns SUCCESS if ALL children succeed. Returns FAILURE immediately if any child fails. Returns RUNNING if a child returns RUNNING.
   * - Selector (?)
     - Executes children left-to-right. Returns SUCCESS immediately if any child succeeds. Returns FAILURE if ALL children fail. Returns RUNNING if a child returns RUNNING.
   * - Condition
     - Checks a condition (e.g., "Is lane clear?"). Returns SUCCESS or FAILURE. Never returns RUNNING.
   * - Action
     - Performs an action (e.g., "Set target lane"). May return RUNNING if action takes multiple ticks.

**Why This Matters:**

Behavior Trees are used in production autonomous driving systems because they provide a clear, maintainable way to encode complex driving policies. Understanding BT design is essential for building safe, reliable self-driving vehicles.

---------------------------------------------------------
3. System Architecture
---------------------------------------------------------

The system follows a hierarchical architecture where each module processes information and passes commands to the next level.

Data Flow
~~~~~~~~~

.. list-table::
   :widths: 25 75
   :header-rows: 1
   :class: compact-table

   * - **Module**
     - **Description**
   * - **Perception (Provided)**
     - Reads ego vehicle state, obstacle positions, and lane information from CARLA
   * - **Behavioral Planner (You Implement)**
     - Uses a Behavior Tree to decide what maneuver to execute
   * - **Trajectory Planner (Provided)**
     - Generates smooth, collision-free trajectory to execute the maneuver
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
     - ✓ (This Assignment)
   * - Behavioral Planner (``behavioral_planner.py``)
     - 
     - ✓ (This Assignment)
   * - Frenet Transform (``frenet_transform.py``)
     - 
     - ✓ (Final Project)
   * - Polynomial Trajectory (``polynomial_trajectory.py``)
     - 
     - ✓ (Final Project)
   * - Trajectory Planner (``trajectory_planner.py``)
     - 
     - ✓ (Final Project)
   * - Controller (``controller.py``)
     - ✓
     - 
   * - Visualization (``visualization.py``)
     - ✓
     - 


Key Interfaces
~~~~~~~~~~~~~~

**Perception → Behavioral Planner (via Blackboard):**

The Blackboard is a shared data structure that stores perception data and behavioral commands. Your nodes will read from and write to the blackboard.

.. code-block:: python

   # Data available on the blackboard
   blackboard.ego_state      # Dict: {x, y, theta, v, lane_id}
   blackboard.obstacles      # List of Dicts: [{x, y, vx, vy, lane_id}, ...]
   blackboard.lane_info      # Dict: {current_lane, left_lane_exists, right_lane_exists, lane_width}
   blackboard.config         # Dict: Configuration parameters
   blackboard.route          # List: [target_lane_sequence]

**Behavioral Planner → Trajectory Planner:**

Your planner outputs a ``BehavioralCommand`` that tells the trajectory planner what maneuver to execute:

.. code-block:: python

   BehavioralCommand:
       maneuver: str       # 'lane_keep' | 'follow' | 'lane_change_left' | 'lane_change_right' | 'stop'
       target_lane: int    # Target lane ID
       target_speed: float # Target speed in m/s

---------------------------------------------------------
4. Configuration Parameters
---------------------------------------------------------

The behavioral planner uses parameters defined in ``config/planner_config.yaml``. Understanding these parameters is essential for implementing the condition nodes correctly.

Behavioral Planner Parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 30 15 55
   :header-rows: 1
   :class: compact-table

   * - **Parameter**
     - **Default**
     - **Description**
   * - ``target_speed``
     - 25.0 m/s
     - Cruising speed when lane keeping. This is the desired speed when no obstacles are present (~90 km/h).
   * - ``safe_follow_distance``
     - 20.0 m
     - Minimum distance to maintain behind the lead vehicle. If the ego vehicle is closer than this, it should follow the lead vehicle rather than maintain cruising speed.
   * - ``lane_change_min_gap``
     - 25.0 m
     - Minimum gap required in the target lane (both front and rear) before a lane change is considered safe.
   * - ``stop_distance``
     - 10.0 m
     - Distance at which to stop before a blocking (stationary) obstacle. Used by ``IsObstacleBlocking`` condition.
   * - ``slow_vehicle_threshold``
     - 18.0 m/s
     - Speed threshold to consider a vehicle as "slow". If the lead vehicle is below this speed and the ego wants to go faster, a lane change may be desirable.

**Example Configuration:**

.. code-block:: yaml

   behavioral_planner:
     target_speed: 25.0           # m/s (~90 km/h)
     safe_follow_distance: 20.0   # meters
     lane_change_min_gap: 25.0    # meters
     stop_distance: 10.0          # meters
     slow_vehicle_threshold: 18.0 # m/s

These parameters are accessed in your nodes via the blackboard:

.. code-block:: python

   # In a condition or action node
   config = blackboard.get('config')
   stop_distance = config['stop_distance']
   safe_follow_distance = config['safe_follow_distance']

---------------------------------------------------------
5. Assignment Tasks
---------------------------------------------------------

You will implement **condition nodes**, **action nodes**, and **assemble the behavior tree** in the provided starter package.

Required Behavior Tree Structure
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You must implement the following tree structure. The root is a **Selector** that tries each driving behavior in priority order:

.. code-block:: text

   Root [Selector]
   ├── Emergency Stop [Sequence]
   │   ├── [Condition] IsObstacleBlocking?
   │   └── [Action] SetStopCommand
   ├── Lane Change [Sequence]
   │   ├── [Condition] ShouldChangeLane?
   │   ├── [Condition] IsLaneChangeSafe?
   │   └── [Action] SetLaneChangeCommand
   ├── Follow Vehicle [Sequence]
   │   ├── [Condition] IsVehicleAhead?
   │   ├── [Condition] IsTooClose?
   │   └── [Action] SetFollowCommand
   └── Lane Keep [Sequence]
       └── [Action] SetLaneKeepCommand

**Execution Logic:** The Selector tries each branch in order. The first branch to return SUCCESS determines the behavior. If a branch fails (condition not met), the Selector tries the next branch. The LaneKeep branch always succeeds, serving as the default behavior.

Task 1: Implement Helper Functions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**File:** ``bt_nodes.py``

Implement the following helper functions that will be used by your condition nodes:

.. prf:algorithm:: get_lead_vehicle
   :label: get-lead-vehicle

   **Inputs:**
   
   - :math:`ego` (ego vehicle state with x, y, theta, v, lane_id)
   - :math:`obstacles` (list of obstacle dictionaries)
   
   **Output:** The closest obstacle ahead in the same lane, or None

   1. :math:`lead \gets \text{None}`
   
   2. :math:`min\_dist \gets \infty`
   
   3. **for** each :math:`obs` **in** :math:`obstacles` **do**
   
      a. **if** :math:`obs.lane\_id \neq ego.lane\_id` **then continue**
      
      b. :math:`d_{long} \gets \text{compute_longitudinal_distance}(ego, obs)`
      
      c. **if** :math:`d_{long} > 0` **and** :math:`d_{long} < min\_dist` **then**
      
         i. :math:`min\_dist \gets d_{long}`
         
         ii. :math:`lead \gets obs`
   
   4. **return** :math:`lead`


.. prf:algorithm:: compute_longitudinal_distance
   :label: compute-longitudinal-distance

   **Inputs:**
   
   - :math:`ego` (ego vehicle state with x, y, theta)
   - :math:`obs` (obstacle with x, y)
   
   **Output:** Signed longitudinal distance (positive = ahead, negative = behind)

   1. :math:`dx \gets obs.x - ego.x`
   
   2. :math:`dy \gets obs.y - ego.y`
   
   3. :math:`d_{long} \gets dx \cdot \cos(ego.\theta) + dy \cdot \sin(ego.\theta)`
   
   4. **return** :math:`d_{long}`


.. prf:algorithm:: check_gap_in_lane
   :label: check-gap-in-lane

   **Inputs:**
   
   - :math:`ego` (ego vehicle state)
   - :math:`obstacles` (list of obstacles)
   - :math:`target\_lane` (lane ID to check)
   - :math:`min\_gap` (minimum required gap in meters)
   
   **Output:** True if gap is sufficient, False otherwise

   1. **for** each :math:`obs` **in** :math:`obstacles` **do**
   
      a. **if** :math:`obs.lane\_id \neq target\_lane` **then continue**
      
      b. :math:`d_{long} \gets \text{compute_longitudinal_distance}(ego, obs)`
      
      c. **if** :math:`|d_{long}| < min\_gap` **then return** False
   
   2. **return** True


Task 2: Implement Condition Nodes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**File:** ``bt_nodes.py``

Implement the ``evaluate()`` method for each condition node. Each method receives the blackboard and returns ``True`` (SUCCESS) or ``False`` (FAILURE).


.. prf:algorithm:: IsObstacleBlocking
   :label: is-obstacle-blocking

   **Inputs:**
   
   - :math:`bb` (blackboard with ego_state, obstacles, config)
   
   **Output:** True if stationary obstacle blocks the lane

   1. :math:`ego \gets bb.ego\_state`
   
   2. :math:`obstacles \gets bb.obstacles`
   
   3. :math:`d_{stop} \gets bb.config['stop\_distance']`
   
   4. **for** each :math:`obs` **in** :math:`obstacles` **do**
   
      a. **if** :math:`obs.lane\_id \neq ego.lane\_id` **then continue**
      
      b. :math:`v_{obs} \gets \sqrt{obs.vx^2 + obs.vy^2}`
      
      c. **if** :math:`v_{obs} > 0.5` **then continue** (not stationary)
      
      d. :math:`d_{long} \gets \text{compute_longitudinal_distance}(ego, obs)`
      
      e. **if** :math:`d_{long} > 0` **and** :math:`d_{long} < d_{stop}` **then return** True
   
   5. **return** False


.. prf:algorithm:: IsVehicleAhead
   :label: is-vehicle-ahead

   **Inputs:**
   
   - :math:`bb` (blackboard with ego_state, obstacles)
   
   **Output:** True if any vehicle is ahead in the same lane

   1. :math:`lead \gets \text{get_lead_vehicle}(bb.ego\_state, bb.obstacles)`
   
   2. **return** :math:`lead \neq \text{None}`


.. prf:algorithm:: IsTooClose
   :label: is-too-close

   **Inputs:**
   
   - :math:`bb` (blackboard with ego_state, obstacles, config)
   
   **Output:** True if distance to lead vehicle is less than safe following distance

   1. :math:`lead \gets \text{get_lead_vehicle}(bb.ego\_state, bb.obstacles)`
   
   2. **if** :math:`lead = \text{None}` **then return** False
   
   3. :math:`d_{safe} \gets bb.config['safe\_follow\_distance']`
   
   4. :math:`dist \gets \text{compute_longitudinal_distance}(bb.ego\_state, lead)`
   
   5. **return** :math:`dist < d_{safe}`


.. prf:algorithm:: ShouldChangeLane
   :label: should-change-lane

   **Inputs:**
   
   - :math:`bb` (blackboard with ego_state, obstacles, route, config)
   
   **Output:** True if lane change is desirable

   1. :math:`ego \gets bb.ego\_state`
   
   2. :math:`route \gets bb.get('route')`
   
   3. **if** :math:`route` is not empty **and** :math:`route[0] \neq ego.lane\_id` **then**
   
      a. **return** True (route requires different lane)
   
   4. :math:`lead \gets \text{get_lead_vehicle}(ego, bb.obstacles)`
   
   5. **if** :math:`lead \neq \text{None}` **then**
   
      a. :math:`v_{lead} \gets \sqrt{lead.vx^2 + lead.vy^2}`
      
      b. :math:`v_{slow} \gets bb.config['slow\_vehicle\_threshold']`
      
      c. **if** :math:`v_{lead} < v_{slow}` **then return** True
   
   6. **return** False


.. prf:algorithm:: IsLaneChangeSafe
   :label: is-lane-change-safe

   **Inputs:**
   
   - :math:`bb` (blackboard with ego_state, obstacles, lane_info, config)
   
   **Output:** True if safe to change lanes

   1. :math:`ego \gets bb.ego\_state`
   
   2. :math:`lane\_info \gets bb.lane\_info`
   
   3. :math:`min\_gap \gets bb.config['lane\_change\_min\_gap']`
   
   4. Determine target lane (prefer left for passing):
   
      a. **if** :math:`lane\_info['left\_lane\_exists']` **then** :math:`target \gets ego.lane\_id + 1`
      
      b. **else if** :math:`lane\_info['right\_lane\_exists']` **then** :math:`target \gets ego.lane\_id - 1`
      
      c. **else return** False (no lane available)
   
   5. :math:`bb.set('target\_lane\_for\_change', target)`
   
   6. **return** :math:`\text{check_gap_in_lane}(ego, bb.obstacles, target, min\_gap)`


Task 3: Implement Action Nodes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**File:** ``bt_nodes.py``

Implement the ``execute()`` method for each action node. Each method receives the blackboard and returns a ``NodeStatus``.


.. prf:algorithm:: SetStopCommand
   :label: set-stop-command

   **Inputs:**
   
   - :math:`bb` (blackboard)
   
   **Output:** NodeStatus.SUCCESS

   1. :math:`ego \gets bb.ego\_state`
   
   2. :math:`cmd \gets \text{BehavioralCommand}()`
   
   3. :math:`cmd.maneuver \gets \text{'stop'}`
   
   4. :math:`cmd.target\_lane \gets ego.lane\_id`
   
   5. :math:`cmd.target\_speed \gets 0.0`
   
   6. :math:`bb.behavioral\_command \gets cmd`
   
   7. **return** NodeStatus.SUCCESS


.. prf:algorithm:: SetLaneKeepCommand
   :label: set-lane-keep-command

   **Inputs:**
   
   - :math:`bb` (blackboard)
   
   **Output:** NodeStatus.SUCCESS

   1. :math:`ego \gets bb.ego\_state`
   
   2. :math:`v_{target} \gets bb.config['target\_speed']`
   
   3. :math:`cmd \gets \text{BehavioralCommand}()`
   
   4. :math:`cmd.maneuver \gets \text{'lane_keep'}`
   
   5. :math:`cmd.target\_lane \gets ego.lane\_id`
   
   6. :math:`cmd.target\_speed \gets v_{target}`
   
   7. :math:`bb.behavioral\_command \gets cmd`
   
   8. **return** NodeStatus.SUCCESS


.. prf:algorithm:: SetFollowCommand
   :label: set-follow-command

   **Inputs:**
   
   - :math:`bb` (blackboard)
   
   **Output:** NodeStatus.SUCCESS

   1. :math:`ego \gets bb.ego\_state`
   
   2. :math:`lead \gets \text{get_lead_vehicle}(ego, bb.obstacles)`
   
   3. **if** :math:`lead \neq \text{None}` **then**
   
      a. :math:`v_{lead} \gets \sqrt{lead.vx^2 + lead.vy^2}`
   
   4. **else**
   
      a. :math:`v_{lead} \gets bb.config['target\_speed']`
   
   5. :math:`cmd \gets \text{BehavioralCommand}()`
   
   6. :math:`cmd.maneuver \gets \text{'follow'}`
   
   7. :math:`cmd.target\_lane \gets ego.lane\_id`
   
   8. :math:`cmd.target\_speed \gets v_{lead}`
   
   9. :math:`bb.behavioral\_command \gets cmd`
   
   10. **return** NodeStatus.SUCCESS


.. prf:algorithm:: SetLaneChangeCommand
   :label: set-lane-change-command

   **Inputs:**
   
   - :math:`bb` (blackboard)
   
   **Output:** NodeStatus.RUNNING while lane change in progress, SUCCESS when complete

   1. :math:`ego \gets bb.ego\_state`
   
   2. :math:`in\_progress \gets bb.get('lane\_change\_in\_progress', \text{False})`
   
   3. **if not** :math:`in\_progress` **then**
   
      a. :math:`target \gets bb.get('target\_lane\_for\_change')`
      
      b. :math:`bb.set('lane\_change\_in\_progress', \text{True})`
      
      c. :math:`bb.set('lane\_change\_target', target)`
   
   4. :math:`target \gets bb.get('lane\_change\_target')`
   
   5. **if** :math:`target > ego.lane\_id` **then** :math:`maneuver \gets \text{'lane_change_left'}`
   
   6. **else** :math:`maneuver \gets \text{'lane_change_right'}`
   
   7. :math:`cmd \gets \text{BehavioralCommand}()`
   
   8. :math:`cmd.maneuver \gets maneuver`
   
   9. :math:`cmd.target\_lane \gets target`
   
   10. :math:`cmd.target\_speed \gets bb.config['target\_speed']`
   
   11. :math:`bb.behavioral\_command \gets cmd`
   
   12. **if** :math:`ego.lane\_id = target` **then** (lane change complete)
   
       a. :math:`bb.set('lane\_change\_in\_progress', \text{False})`
       
       b. **return** NodeStatus.SUCCESS
   
   13. **return** NodeStatus.RUNNING


Task 4: Assemble the Behavior Tree
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**File:** ``behavioral_planner.py``

Implement the ``_build_tree()`` method to construct the complete Behavior Tree.

.. prf:algorithm:: BehavioralPlanner._build_tree
   :label: build-tree

   **Output:** BehaviorTree with the decision structure

   1. Create Emergency Stop sequence:
   
      a. :math:`stop\_seq \gets \text{Sequence}(\text{"EmergencyStop"}, [\text{IsObstacleBlocking}(), \text{SetStopCommand}()])`
   
   2. Create Lane Change sequence:
   
      a. :math:`lc\_seq \gets \text{Sequence}(\text{"LaneChange"}, [\text{ShouldChangeLane}(), \text{IsLaneChangeSafe}(), \text{SetLaneChangeCommand}()])`
   
   3. Create Follow Vehicle sequence:
   
      a. :math:`follow\_seq \gets \text{Sequence}(\text{"FollowVehicle"}, [\text{IsVehicleAhead}(), \text{IsTooClose}(), \text{SetFollowCommand}()])`
   
   4. Create Lane Keep sequence:
   
      a. :math:`keep\_seq \gets \text{Sequence}(\text{"LaneKeep"}, [\text{SetLaneKeepCommand}()])`
   
   5. Create Root selector:
   
      a. :math:`root \gets \text{Selector}(\text{"Root"}, [stop\_seq, lc\_seq, follow\_seq, keep\_seq])`
   
   6. **return** :math:`\text{BehaviorTree}(root)`


---------------------------------------------------------
6. Test Scenarios
---------------------------------------------------------

Your implementation will be tested on three scenarios of increasing complexity.

Scenario 1: Lane Keeping
~~~~~~~~~~~~~~~~~~~~~~~~~

**Description:** Straight highway segment with no obstacles. Maintain lane at constant speed.

**What It Tests:**

- BT defaults to LaneKeep when no other conditions are met
- SetLaneKeepCommand sets correct target speed and lane
- Tree structure correctly falls through to default behavior

**Success Criteria:**

- Maneuver output: ``lane_keep``
- Target speed: ``target_speed`` from config (25 m/s)
- Target lane: Current lane maintained

Scenario 2: Vehicle Following
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Description:** Lead vehicle ahead traveling at varying speeds (15-25 m/s). Ego vehicle must follow safely.

**What It Tests:**

- IsVehicleAhead correctly detects lead vehicle
- IsTooClose correctly evaluates following distance
- SetFollowCommand matches lead vehicle speed

**Success Criteria:**

- Maneuver output: ``follow`` when close to lead vehicle
- Target speed: Matches lead vehicle speed
- Correct transition between lane_keep and follow

Scenario 3: Lane Change
~~~~~~~~~~~~~~~~~~~~~~~~

**Description:** Slow vehicle ahead (15 m/s) while ego wants to travel at 25 m/s. Adjacent lane has gap for passing.

**What It Tests:**

- ShouldChangeLane detects slow vehicle ahead
- IsLaneChangeSafe checks gap in target lane
- SetLaneChangeCommand returns RUNNING during maneuver
- Correct transition back to lane_keep after completion

**Success Criteria:**

- Maneuver output: ``lane_change_left`` or ``lane_change_right``
- RUNNING state maintained during lane change
- SUCCESS returned when lane change completes

---------------------------------------------------------
7. Getting Started
---------------------------------------------------------

Getting Started Checklist
~~~~~~~~~~~~~~~~~~~~~~~~~~

1. ☐ CARLA simulator installed and running (version 0.9.13 or later)
2. ☐ Starter code downloaded and Python environment configured
3. ☐ Can run the provided controller demo and see vehicle moving
4. ☐ Understand the Behavior Tree framework (review ``bt_framework.py``)
5. ☐ Understand the Blackboard data structure
6. ☐ Review L5 (Behavioral Planning - Behavior Trees) lecture materials

Step 1: Understand the Framework
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Review ``src/behavior_tree/__init__.py`` for BT classes
2. Study the ``Blackboard``, ``Sequence``, ``Selector`` classes
3. Understand ``ConditionNode`` and ``ActionNode`` base classes
4. Run the provided BT example to see tick mechanism

Step 2: Implement Helper Functions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Open ``src/behavior_tree/bt_nodes.py``
2. Implement ``get_lead_vehicle()``
3. Implement ``compute_longitudinal_distance()``
4. Implement ``check_gap_in_lane()``
5. Test with ``python -m tests.test_bt_nodes``

Step 3: Implement Condition Nodes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Implement ``IsObstacleBlocking.evaluate()``
2. Implement ``IsVehicleAhead.evaluate()``
3. Implement ``IsTooClose.evaluate()``
4. Implement ``ShouldChangeLane.evaluate()``
5. Implement ``IsLaneChangeSafe.evaluate()``
6. Test with unit tests

Step 4: Implement Action Nodes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Implement ``SetStopCommand.execute()``
2. Implement ``SetLaneKeepCommand.execute()``
3. Implement ``SetFollowCommand.execute()``
4. Implement ``SetLaneChangeCommand.execute()`` (handle RUNNING state!)
5. Test with unit tests

Step 5: Assemble the Behavior Tree
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Open ``src/behavioral_planner.py``
2. Implement ``_build_tree()`` method
3. Verify tree structure with ``planner.print_tree()``
4. Test with mock perception data

Step 6: Integration Testing
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. Run Scenario 1 (Lane Keeping)
2. Run Scenario 2 (Vehicle Following)
3. Run Scenario 3 (Lane Change)

.. code-block:: bash

   # Start CARLA simulator first (in separate terminal)
   ./CarlaUE4.sh
   
   # Run scenarios
   python -m scenarios.scenario_runner --scenario highway --test lane_keep
   python -m scenarios.scenario_runner --scenario highway --test follow
   python -m scenarios.scenario_runner --scenario highway --test lane_change

---------------------------------------------------------
8. Provided Package Structure
---------------------------------------------------------

.. code-block:: text

   rwa3_final_starter/
   ├── src/
   │   ├── behavior_tree/
   │   │   ├── __init__.py              # BT framework classes (Provided)
   │   │   ├── bt_framework.py          # Re-exports (Provided)
   │   │   └── bt_nodes.py              # <-- TODO: Implement nodes
   │   ├── behavioral_planner.py        # <-- TODO: Implement tree assembly
   │   ├── carla_interface.py           # CARLA connection (Provided)
   │   ├── perception.py                # Perception module (Provided)
   │   ├── trajectory_planner.py        # Trajectory planner (Final Project)
   │   ├── frenet_transform.py          # Frenet coordinates (Final Project)
   │   ├── polynomial_trajectory.py     # Polynomial generation (Final Project)
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

- ✏️ ``src/behavior_tree/bt_nodes.py`` — Implement condition and action nodes
- ✏️ ``src/behavioral_planner.py`` — Implement tree assembly

**Files you should NOT modify:**

- ❌ ``src/behavior_tree/__init__.py`` — Provided BT framework
- ❌ ``src/carla_interface.py`` — Provided CARLA interface
- ❌ ``src/perception.py`` — Provided perception module
- ❌ ``src/controller.py`` — Provided controller
- ❌ ``scenarios/*`` — Provided test scenarios

---------------------------------------------------------
9. Testing and Evaluation
---------------------------------------------------------

Run Unit Tests
~~~~~~~~~~~~~~~

.. code-block:: bash

   # Test BT nodes
   python -m tests.test_bt_nodes
   
   # Expected output shows PASSED/FAILED for each test

Run Integration Tests
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Test with CARLA
   python -m scenarios.scenario_runner --scenario highway --test lane_keep --visualize
   python -m scenarios.scenario_runner --scenario highway --test follow --visualize
   python -m scenarios.scenario_runner --scenario highway --test lane_change --visualize

Evaluation Criteria
~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :widths: 30 70
   :header-rows: 1
   :class: compact-table

   * - **Metric**
     - **Requirement**
   * - **Condition Nodes**
     - Return correct SUCCESS/FAILURE for all test cases
   * - **Action Nodes**
     - Set correct BehavioralCommand fields
   * - **Tree Structure**
     - Correct priority ordering of behaviors
   * - **RUNNING State**
     - Lane change correctly uses RUNNING until complete
   * - **Integration**
     - All scenarios pass without errors

---------------------------------------------------------
10. Submission Requirements
---------------------------------------------------------

What to Submit
~~~~~~~~~~~~~~

Submit a **single ZIP file** named ``groupX_rwa3.zip`` containing:

.. code-block:: text

   groupX_rwa3/
   ├── src/
   │   ├── behavior_tree/
   │   │   └── bt_nodes.py              # Your implementation
   │   └── behavioral_planner.py        # Your implementation
   ├── results/
   │   ├── scenario_lane_keep.png       # Screenshot
   │   ├── scenario_follow.png          # Screenshot
   │   ├── scenario_lane_change.png     # Screenshot
   │   └── test_output.txt              # Unit test results
   └── README.md                        # Brief documentation

README Requirements
~~~~~~~~~~~~~~~~~~~~

Your ``README.md`` must include:

1. **Team Members:** Names and UIDs
2. **Build/Run Instructions:** How to run your code
3. **Implementation Notes:** Key design decisions, challenges faced
4. **Test Results:** Summary of passing/failing tests

Submission Checklist
~~~~~~~~~~~~~~~~~~~~

Before submitting, ensure:

- ☐ All condition nodes implemented and tested
- ☐ All action nodes implemented and tested
- ☐ ``_build_tree()`` creates correct tree structure
- ☐ RUNNING state handled correctly for lane change
- ☐ All three scenarios run without errors
- ☐ Screenshots from each scenario included
- ☐ Unit test output saved
- ☐ Code is well-commented
- ☐ ZIP file follows naming convention

---------------------------------------------------------
11. Grading Rubric
---------------------------------------------------------

**Total: 40 points**

**Condition Nodes (15 pts):**

- **IsObstacleBlocking (3 pts):**
  
  - 3 pts: Correctly detects stationary obstacles within stop_distance
  - 2 pts: Works but edge cases fail
  - 0 pts: Not functional

- **IsVehicleAhead (2 pts):**
  
  - 2 pts: Correctly identifies lead vehicle in same lane
  - 1 pt: Partially correct
  - 0 pts: Not functional

- **IsTooClose (3 pts):**
  
  - 3 pts: Correctly compares distance to safe_follow_distance
  - 2 pts: Distance calculation has minor issues
  - 0 pts: Not functional

- **ShouldChangeLane (4 pts):**
  
  - 4 pts: Correctly evaluates route and slow vehicle conditions
  - 2 pts: One condition works
  - 0 pts: Not functional

- **IsLaneChangeSafe (3 pts):**
  
  - 3 pts: Correctly checks gap in target lane
  - 2 pts: Gap check works but target lane selection wrong
  - 0 pts: Not functional

**Action Nodes (15 pts):**

- **SetStopCommand (2 pts):**
  
  - 2 pts: Sets maneuver='stop', target_speed=0
  - 1 pt: Partially correct
  - 0 pts: Not functional

- **SetLaneKeepCommand (3 pts):**
  
  - 3 pts: Sets correct maneuver, lane, and speed
  - 2 pts: Minor issues
  - 0 pts: Not functional

- **SetFollowCommand (4 pts):**
  
  - 4 pts: Correctly matches lead vehicle speed
  - 2 pts: Sets follow but wrong speed
  - 0 pts: Not functional

- **SetLaneChangeCommand (6 pts):**
  
  - 6 pts: Correct RUNNING→SUCCESS transition, correct maneuver direction
  - 4 pts: Lane change works but RUNNING not handled
  - 2 pts: Partially functional
  - 0 pts: Not functional

**Tree Assembly (5 pts):**

- **Correct structure (3 pts):**
  
  - 3 pts: All branches in correct priority order
  - 2 pts: Minor structure issues
  - 0 pts: Incorrect structure

- **Integration (2 pts):**
  
  - 2 pts: Tree ticks correctly, blackboard updates work
  - 1 pt: Minor integration issues
  - 0 pts: Tree doesn't execute

**Integration & Documentation (5 pts):**

- **Scenarios pass (3 pts):**
  
  - 3 pts: All 3 scenarios complete without errors
  - 2 pts: 2 scenarios pass
  - 1 pt: 1 scenario passes
  - 0 pts: No scenarios pass

- **Documentation (2 pts):**
  
  - 2 pts: Complete README, well-commented code
  - 1 pt: Partial documentation
  - 0 pts: Missing documentation

---------------------------------------------------------
12. Learning Outcomes
---------------------------------------------------------

By completing this assignment, you will:

- ✅ **Understand Behavior Tree architectures**
  
  - Node types and their semantics (Sequence, Selector, Condition, Action)
  - Tick mechanism and status propagation
  - Tree structure for driving behaviors

- ✅ **Implement condition-based decision making**
  
  - Evaluate driving situations from perception data
  - Design modular, reusable condition nodes
  - Handle edge cases and boundary conditions

- ✅ **Handle multi-tick actions**
  
  - Use RUNNING state for extended actions
  - Track action progress through the blackboard
  - Correctly transition to SUCCESS/FAILURE

- ✅ **Integrate with a complete AV stack**
  
  - Interface with perception modules
  - Output commands for trajectory planning
  - Understand hierarchical planning architecture

**Connection to Course Material:**

This assignment implements concepts from L5 (Behavioral Planning):

- Behavior Trees as decision-making architectures
- Modularity and composition of driving behaviors
- Interface between behavioral and trajectory planning

You're building the **decision-making brain** of an autonomous vehicle!

---------------------------------------------------------
13. References
---------------------------------------------------------

**Primary References:**

- Colledanchise, M. & Ögren, P. (2018). *Behavior Trees in Robotics and AI: An Introduction.* CRC Press.
- Behavior Trees in Games: https://www.behaviortree.dev/

**Documentation:**

- CARLA Simulator: https://carla.readthedocs.io/

**Additional Reading:**

- Paden, B., et al. (2016). *A Survey of Motion Planning and Control Techniques Adopted in Self-Driving Vehicles.* IEEE Transactions on Intelligent Vehicles.
