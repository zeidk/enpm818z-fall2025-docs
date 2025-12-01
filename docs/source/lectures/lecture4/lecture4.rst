.. _l4-trajectory-prediction-overview:

L4 - Trajectory Prediction
==================================================

This site accompanies *ENPM818Z - Lecture 4: Trajectory Prediction for Automated Vehicles*.

.. admonition:: Resources
   :class: resources


   🔗 `Lecture 4 slides (v1.0) <https://drive.google.com/file/d/1KMSkZt5qy9PLPnDXkWwUQea9bnF365zr/view?usp=sharing>`_


Overview
--------

**Bridge: From Perception to Action**

Perception tells us the world as it is now: "A car at position (x,y) with velocity v."
Prediction answers the critical question: "Where will that car be in 3 seconds?"

The future depends on our actions. If we accelerate, nearby vehicles react
differently than if we brake. Planning requires forecasting this interactive future
to ensure safe, efficient trajectories.

Learning Objectives
-------------------

1. Understand the fundamental challenge of predicting future states in dynamic, interactive environments
2. Apply physics-based models to forecast short-term trajectories and understand their limitations
3. Comprehend modern learning-based prediction architectures including RNNs, GNNs, and Transformers
4. Recognize the critical importance of uncertainty quantification and multimodal prediction
5. Design and evaluate intent inference systems for anticipating high-level goals

.. _l4-Prediction_Time_Horizons:

Prediction Time Horizons
-------------------------

Prediction is not a monolithic problem: it operates across multiple time horizons,
each with different requirements, accuracies, and use cases. Understanding
these horizons is essential for selecting appropriate prediction methods and
balancing competing demands of safety, comfort, and computational efficiency.

Short-Term Prediction (0–2 seconds)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Immediate prediction is critical for collision avoidance, especially in
environments with pedestrian traffic.

- **Safety-critical minimum**: Approximately 1.6 seconds is required to ensure safe detection and response timing for pedestrian scenarios [22]_.
- **Lightweight systems**: Several monocular-vision pedestrian tracking and emergency braking systems use 1-second horizons to balance computational efficiency with safety [8]_, [26]_.
- **Advanced control optimization**: Model Predictive Path Integral (MPPI) control can achieve near-optimal trajectory tracking with horizons as short as 0.7 seconds, demonstrating 98.32% reduction in average maneuver cost compared to longer horizons [28]_.

Medium-Term Prediction (3–6 seconds)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This range represents the current practical standard for autonomous driving
applications, balancing reactiveness with motion planning depth.

- **Industry practice**: Autonomous vehicle systems typically anticipate surrounding road user behavior over the next 5 seconds to enable safe decision-making and comfortable motion planning [11]_.
- **Argoverse 2 benchmark**: Uses 5 seconds of observation history to predict 6 seconds into the future, establishing a widely-adopted standard for model evaluation [19]_.
- **Waymo Open Dataset**: The end-to-end driving challenge requires forecasting 5-second trajectory waypoints, reinforcing this time window as operationally meaningful [27]_.

Long-Term Prediction (7–15 seconds)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Long-horizon forecasting supports smoother driving and tactical planning,
though predictive uncertainty increases significantly.

- **7–8 second horizons**: Improve maneuver smoothness and navigation efficiency in dynamic environments [23]_, [24]_.
- **10–15 second horizons**: Enable smoother trajectories and reduced acceleration variance, improving passenger comfort [20]_, [22]_.
- **Limitations beyond 10 seconds**: Predictions become reliable only in structured scenarios (e.g., straight freeway driving) and require long-range scene understanding beyond the immediate field of view [25]_, [27]_.

Key Insights
^^^^^^^^^^^^

No single prediction method effectively handles all time horizons.

- **Physics-based models** excel at short-term prediction (0–2s) where dynamics dominate
- **Learning-based models** capture intent and interactions in the medium-term (3–6s)
- **Long-term prediction** (7–15s) remains challenging due to compounding uncertainty and the need for high-level contextual reasoning

The Prediction Challenge
-------------------------

**How do we forecast where dynamic agents will be?**

Trajectory prediction forecasts the future movement of dynamic agents: one of
the most challenging tasks in autonomous driving because human behavior is
difficult to model.

Requirements for prediction systems:

- **Interactive**: Account for how the AV's actions influence other agents
- **Multimodal**: Represent multiple possible futures (turn left OR go straight)
- **Probabilistic**: Assign likelihoods to different outcomes
- **Contextual**: Incorporate map data, traffic rules, and social norms


.. _l4-physics_models:

Physics-Based Prediction Models
--------------------------------

Physics-based models assume agents follow simple kinematic or dynamic
rules. They are computationally efficient and provide immediate predictions,
making them ideal for short-term horizons (0–2 seconds), but cannot capture
intent or complex decision-making.

Common implementations: Kalman Filters (KF) and Extended Kalman Filters
(EKF) track an object's state and extrapolate motion forward.

Hierarchy of Motion Models
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Motion models for trajectory prediction organized by increasing complexity:

**Level 1: Point Mass (CV, CA)**

- Agent as point, no orientation; motion in any direction
- Fastest computation, minimal state

**Level 2: Oriented (CTRV, CTRA)**

- Includes heading; motion respects orientation
- Better for directional constraints

**Level 3: Kinematic Vehicle**

- Enforces non-holonomic constraints (bicycle model)
- Accounts for wheelbase, steering limits

**Level 4: Dynamic**

- Includes tire forces, slip, load transfer
- High cost; rarely used for real-time prediction

.. important::
   **Design Principle**: Use the simplest model justified by available measurements.

Constant Velocity (CV) Model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Level 1: Point Mass**

Assumes the object continues in a straight line at current velocity.

**State Update**:

.. math::

   x(t + \Delta t) = x(t) + v(t) \cdot \Delta t
   
   v(t + \Delta t) = v(t)

- **Strengths**: Minimal computation; useful for very short-term predictions (0–1s) and distant objects
- **Limitations**: Fails for turns, stops, or acceleration changes
- **Optimal Horizon**: 0–1 second for straight-line, constant-speed motion

Constant Acceleration (CA) Model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Level 1: Point Mass**

Assumes the object maintains current acceleration over time.

**State Update**:

.. math::

   x(t + \Delta t) = x(t) + v(t) \cdot \Delta t + \frac{1}{2}a(t) \cdot \Delta t^2
   
   v(t + \Delta t) = v(t) + a(t) \cdot \Delta t

- **Strengths**: Fast computation; handles steady acceleration/deceleration
- **Limitations**: Fails when acceleration changes (turns, gradual stops); constant acceleration assumption rarely holds
- **Optimal Horizon**: 0–1 second for steady acceleration or braking scenarios

Constant Turn Rate and Velocity (CTRV) Model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Level 2: Oriented**

Assumes the agent follows a circular arc at constant speed and turn rate.

**State Vector**: :math:`\mathbf{x} = [x, y, \psi, v, \omega]^\top` where :math:`\psi` is heading and :math:`\omega` is turn rate

**State Update** (if :math:`\omega \neq 0`):

.. math::

   x(t + \Delta t) = x(t) + \frac{v}{\omega}[\sin(\psi + \omega\Delta t) - \sin(\psi)]
   
   y(t + \Delta t) = y(t) + \frac{v}{\omega}[\cos(\psi) - \cos(\psi + \omega\Delta t)]
   
   \psi(t + \Delta t) = \psi + \omega \cdot \Delta t

- **Strengths**: Handles curves and intersections; captures non-holonomic vehicle constraints
- **Limitations**: Fails when turn rate or speed changes during maneuver
- **Use Cases**: Highway curves, intersection approaches, baseline for complex models
- **Optimal Horizon**: 1–2 seconds for curved trajectories

Constant Turn Rate and Acceleration (CTRA) Model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Level 2: Oriented**

Assumes the agent follows a circular arc with constant turn rate while
accelerating or decelerating.

**State Vector**: :math:`\mathbf{x} = [x, y, \psi, v, \omega, a]^\top` where :math:`\psi` is heading, :math:`\omega` is turn rate, :math:`a` is longitudinal acceleration

**State Update** (if :math:`\omega \neq 0`):

.. math::

   v(t + \Delta t) = v + a \cdot \Delta t
   
   x(t + \Delta t) = x + \frac{1}{\omega}[(v + a\Delta t)\sin(\psi + \omega\Delta t) - v\sin(\psi)]
   
   y(t + \Delta t) = y + \frac{1}{\omega}[v\cos(\psi) - (v + a\Delta t)\cos(\psi + \omega\Delta t)]
   
   \psi(t + \Delta t) = \psi + \omega \cdot \Delta t

- **Strengths**: Handles acceleration/deceleration through curves; realistic for intersections and merging
- **Limitations**: Fails when turn rate or acceleration rate changes
- **Use Cases**: Highway merging, accelerating through curves, turning at intersections
- **Optimal Horizon**: 1–3 seconds for curved trajectories with speed changes


.. _l4-bicycle_model:


Kinematic Bicycle Model
^^^^^^^^^^^^^^^^^^^^^^^^

**Level 3: Kinematic Vehicle**

The most sophisticated physics-based model for vehicle motion prediction,
representing the car as a bicycle with front and rear axles.

Captures fundamental vehicle constraints: no sideways motion, limited turning
radius based on geometry. Accurate enough for short-term prediction while
remaining computationally efficient.

**Key Parameters**:

- :math:`L` = wheelbase
- :math:`\theta` = heading angle
- :math:`\delta` = steering angle
- :math:`v` = velocity

**State**: :math:`\mathbf{x} = [x, y, \theta, v]^\top`

**Equations of Motion**

Rear-Axle Reference (most common for prediction):

.. math::

   \dot{x} = v \cdot \cos(\theta)
   
   \dot{y} = v \cdot \sin(\theta)
   
   \dot{\theta} = \frac{v}{L} \cdot \tan(\delta)

Center-of-Mass Reference: Replace :math:`\theta` with :math:`\theta + \beta` in position equations, where :math:`\beta = \arctan(\frac{l_r}{L}\tan(\delta))` is the body slip angle (geometric heading offset, not tire slip).

- **Key Assumption**: Perfect tire-road contact with no tire slip (pure rolling)
- **Strengths**: Enforces non-holonomic constraints; accurate for normal driving; computationally efficient
- **Limitations**: Ignores tire slip; breaks down at high speeds (>60 mph), aggressive maneuvers, or low-friction conditions
- **Optimal Horizon**: 0–2 seconds for vehicle trajectory prediction

Why Rear-Axle Reference for Prediction?
""""""""""""""""""""""""""""""""""""""""

- **Simpler model**: Eliminates body slip angle :math:`\beta` calculation, reducing from 4 to 3 differential equations
- **Better observability**: Vehicle detection systems typically track the rear of vehicles (bounding boxes, license plates); center of mass is not directly observable
- **Unknown parameters**: Center of mass location varies with vehicle type, cargo, passengers, and fuel level—information unavailable for other vehicles
- **Computational efficiency**: Matters when predicting 20+ vehicles simultaneously in real-time
- **Negligible accuracy loss**: For moderate steering angles in normal driving, the difference between rear-axle and center-of-mass tracking is small compared to other prediction uncertainties (intent, sensor noise)

.. note::
   For control of your own vehicle, center-of-mass reference is often preferred since all parameters are known and controllability is better understood at the CoM.

Applications in Autonomous Driving
""""""""""""""""""""""""""""""""""

Where This Model is Used:

The kinematic bicycle model is fundamental to multiple components of the AV stack:

- **Short-term trajectory prediction** (0–2 seconds): Predicting where other vehicles will be, assuming they continue their current control inputs.
- **Model Predictive Control (MPC)**: The model serves as the internal prediction model. The controller optimizes future steering and acceleration commands to follow a reference trajectory.
- **Trajectory feasibility checking**: During motion planning, candidate trajectories are validated against the kinematic model to ensure they can actually be executed.
- **Simulation and testing**: Used in simulation environments for realistic vehicle behavior without the computational cost of full dynamics.

.. tip::
   For longer-horizon prediction (3+ seconds) or complex interactive scenarios, learning-based models typically outperform physics-based approaches.

Dynamic Models
^^^^^^^^^^^^^^

**Level 4: Dynamic**

Include tire forces, slip angles, and load transfer for high-fidelity simulation.

**Key Components**:

- **Tire forces**: Longitudinal and lateral forces (e.g., Pacejka Magic Formula)
- **Slip angles**: Tire heading vs. travel direction when exceeding friction limits
- **Load transfer**: Weight distribution changes affect tire grip

**State**: :math:`\mathbf{x} = [x, y, \psi, v_x, v_y, \omega, \text{tire states}]^\top`

- **Strengths**: Accurate for high-speed and aggressive maneuvers; captures tire-road interactions
- **Limitations**: High computational cost due to coupled nonlinear dynamics requiring numerical integration
- **Applications**: Vehicle simulation, motorsports, offline optimization, controller validation
- **Why rarely used for prediction**: Cannot observe tire states or parameters for other vehicles; computational cost prohibitive for multi-agent real-time prediction


.. _l4-learning_models:

Learning-Based Prediction Models
---------------------------------

Learning-based models overcome physics-based limitations by learning
complex, non-linear, and interactive behaviors from large-scale real-world
driving datasets. They excel in the medium-term horizon (3–6 seconds) where
human intent and decision-making dominate over pure dynamics.

Typical inputs: Agent trajectory history, surrounding agent histories, and map
context (lanes, intersections, traffic rules)

The Data-Driven Revolution
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Massive real-world driving datasets enable learning models to capture human
behavioral complexity beyond physics-based approaches.

**Training Data Scale**:

- **Waymo**: 20M+ miles
- **Tesla**: 1B+ miles (fleet)
- **Cruise**: 5M+ miles
- Hundreds of interactions per mile
- Enables learning rare events: aggressive lane changes, jaywalking, emergency vehicles

**What Models Learn**:

- **Social norms**: Yielding, merging, right-of-way
- **Regional differences**: City vs. suburban driving styles
- **Driver styles**: Aggressive vs. conservative
- **Context**: School zones, construction, weather

.. tip::
   Learning models augment physics with human behavioral patterns: intent, social interactions, and context-dependent decisions.

Recurrent Neural Networks (RNNs)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Core Idea**: Process sequential data by maintaining an internal hidden state
that gets updated at each time step, allowing the network to remember
past information.

**Why RNNs for trajectory prediction?**

- Handle variable-length trajectory histories
- Capture temporal dependencies (current position depends on past motion)
- Natural fit for sequential prediction problems

**Challenge**: Vanilla RNNs struggle with long sequences due to vanishing gradients. This led to advanced architectures like LSTMs and GRUs.


.. _l4-LSTMs_and_GRUs:


LSTMs and GRUs
""""""""""""""

LSTMs (Long Short-Term Memory) [2]_ and GRUs (Gated Recurrent Units) [4]_ learn
temporal dependencies from observed motion through gating mechanisms that
selectively retain relevant information [5]_, [9]_.

**Encoder-Decoder Architecture**: Encoder processes observed trajectory into
hidden state :math:`h_t`; Decoder generates future predictions conditioned on :math:`h_t`.

- **Strengths**: Captures temporal dependencies, handles variable sequences, end-to-end learning.
- **Key Models**: Social LSTM [5]_, CS-LSTM [9]_, SR-LSTM [12]_.
- **Limitations**: Sequential processing (slow), vanishing gradients, inferior to Transformers [17]_.
- **Optimal Horizon**: 3-5 seconds for single-agent trajectory forecasting.

.. _l4-graph_neural_networks:

Graph Neural Networks (GNNs)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

GNNs model the driving scene as a "social graph" where agents (cars,
pedestrians, cyclists) are nodes and their interactions are edges [14]_, [15]_.

**Message Passing**: Information propagates through the graph via multiple
rounds of message passing, where each node aggregates information from its
neighbors to update its state and generate predictions.

- **Social Context**: Learns complex behaviors like yielding, following, pedestrian-vehicle interactions.
- **Map Context**: Encodes map elements (lanes, stop signs, crosswalks) as nodes to learn map-aware behaviors.
- **Key Innovation**: Attention mechanisms dynamically weight which agents and map features are most relevant [16]_.
- **Advantages**: Explicitly models interactions, permutation invariant, scalable to many agents.
- **Optimal Horizon**: 3-6 seconds for multi-agent interaction scenarios.

.. _l4-transformers:

Transformers
^^^^^^^^^^^^

Transformers have largely replaced RNNs in modern prediction systems [7]_.
Self-attention mechanisms dynamically learn which agents and map elements
most influence each target agent's future behavior [18]_, [21]_.

**Scenario**: Predicting a target agent's trajectory by identifying which scene elements influence its behavior.

**Attention Weights**: Higher weights indicate stronger influence on the target's predicted trajectory.

- Agent 2 (0.6): Highest - directly in path, critical for prediction
- Agent 1 (0.2): Moderate - nearby, influences lane decisions
- Stop Sign (0.15): Context - explains deceleration behavior
- Agent 3 (0.05): Minimal - too far to affect immediate behavior

**Key Advantages**:

- Parallel processing (faster than sequential RNNs)
- Captures long-range temporal dependencies
- Dynamic attention focuses on relevant context
- Handles variable-length sequences

.. _l4-uncertainty_multimodal_prediction:

Uncertainty and Multimodal Prediction
--------------------------------------

The future is not a single, deterministic path: it contains multiple
possible outcomes. A vehicle approaching an intersection might
turn left, turn right, go straight, or stop. Each distinct behavior is
called a **mode**.

Multimodal prediction represents all plausible modes with their
probabilities, rather than committing to a single trajectory. This is
critical for safety: planning against only the most likely future (51%
probability) ignores dangerous alternatives (49% combined).

Uncertainty grows with prediction horizon: a 1-second prediction
has low ambiguity, while a 6-second prediction can diverge into
drastically different trajectories across multiple modes.

Why Multimodality is Critical
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Unimodal Prediction (Dangerous)**:

- Target vehicle approaching intersection
- Predictor outputs: 51% straight (single mode)
- Ignores 49% probability of left turn
- Ego vehicle plans to accelerate through
- Target actually turns left
- **Collision!**

**Multimodal Prediction (Safe)**:

- Target vehicle approaching intersection
- Predictor outputs: 40% straight, 35% left, 25% right
- All modes considered in planning
- Ego vehicle waits until intent is clear
- Safe regardless of target's choice
- **No collision**

.. danger::
   **Critical Safety Principle**: Plan for all plausible futures, not just the most likely one.

.. _l4-Representing_Multimodality:

Representing Multimodality
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Rather than outputting a single "best guess" trajectory, modern prediction
systems output multiple distinct, high-probability future paths:

**K-Best Modes**

Generate K different trajectory hypotheses (e.g., K=6), each representing a distinct intent or maneuver. Each mode receives a probability score, with all probabilities summing to ≤1.0. 

For example, at an intersection:
Mode 1 (straight, 40%), Mode 2 (left turn, 35%), Mode 3 (right turn, 20%), Mode 4 (stop, 5%). 

The number K is chosen to balance coverage of plausible futures against computational cost.

**Probability Distribution per Mode**

Each mode is not a single deterministic trajectory, but rather a probability distribution representing uncertainty around that intent. A "straight" mode might have slight lateral variations, or different speeds within that behavior. This is often visualized as a "cone of uncertainty" that grows over the prediction horizon—narrow near the current time, widening as we predict further into the future.


.. _l4-Quantifying_Uncertainty:

Quantifying Uncertainty
^^^^^^^^^^^^^^^^^^^^^^^^

Each predicted trajectory should include a confidence measure or probability
distribution. This tells the planner how much to trust each prediction and how
conservative to be.

**Aleatoric Uncertainty**

Inherent randomness in human behavior. Even with perfect information, we cannot predict exactly what a human driver will do. This uncertainty grows with prediction horizon.

**Epistemic Uncertainty**

Uncertainty due to model limitations or insufficient training data. This can be reduced with more data and better models, unlike aleatoric uncertainty.

.. _l4-Technical_Approaches_to_Multimodal_Prediction:

Technical Approaches to Multimodal Prediction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


.. _l4-mixture_modal_networks:

Mixture Density Networks (MDNs)
""""""""""""""""""""""""""""""""

MDNs output parameters of a probability distribution rather than single trajectory predictions [1]_, [3]_.

**Inputs**:

- Agent's current state: position (x,y), velocity v, heading θ
- Trajectory history: past positions over previous timesteps
- Context: surrounding agents, map information (lanes, intersections)

**Neural Network Processing**: The network learns to predict multiple possible futures by outputting a mixture of probability distributions (typically Gaussians). Instead of predicting "the car will go here", it predicts "the car might follow path A with 40% probability, path B with 35% probability, or path C with 25% probability".

**Outputs** (for each mode k = 1 to K):

- Mixture weight :math:`\pi_k`: Probability of this mode (e.g., 0.40 for "straight")
- Mean trajectory :math:`\mu_k`: Most likely path for this mode (sequence of future positions)
- Covariance :math:`\Sigma_k`: Uncertainty around this trajectory (how much spread/variance)

.. _l4-Conditional_Variational_Autoencoder:

Conditional Variational Autoencoders (CVAEs)
"""""""""""""""""""""""""""""""""""""""""""""

CVAEs learn a compressed latent representation of possible behaviors [6]_, [16]_.
By sampling from this learned space, the model generates diverse, realistic trajectories.

**Inputs**:

- Past trajectory: Agent's observed motion history
- Scene context: Map features (lanes, intersections) and surrounding agents

**Process**:

- **Encoder**: Compresses past trajectory into latent representation z (behavior space)
- **Sampling**: Draw random samples from latent space to generate diversity
- **Decoder**: Generates future trajectory conditioned on z and scene context

**Outputs**: Multiple diverse future trajectories (sample N times to get N predictions)

Anchor-Based Approaches
""""""""""""""""""""""""

Anchor-based methods pre-define a set of template trajectories representing
common maneuvers [10]_, [13]_. The network then selects and refines the most
appropriate anchors for the current scenario.

**Inputs**:

- Agent's current state and trajectory history
- Scene context (map, surrounding agents)
- Predefined set of K anchor trajectories (e.g., K=3: left lane change, straight, right lane change)

**Network Outputs** (Two-Stage Process):

- **Classification**: Probability distribution over anchors (which maneuver is most likely?)
- **Regression**: Offset corrections to refine each anchor trajectory (fine-tune the template)

**Final Prediction**: Refined trajectories based on most probable anchors


.. _l4-intent_inference:

Intent Inference
----------------

Before predicting the detailed path (trajectory), it is often valuable to first predict
the high-level goal (intent). This hierarchical approach simplifies the problem by
separating "what will they do" from "how will they do it".

Intent inference is particularly valuable for medium to long-term predictions
(3-15 seconds) where understanding the goal significantly constrains the
possible trajectories.

Intent as Classification
^^^^^^^^^^^^^^^^^^^^^^^^

Intent can be formulated as a discrete classification problem with labels such as:

- TURN_LEFT
- TURN_RIGHT
- GO_STRAIGHT
- LANE_CHANGE_LEFT
- LANE_CHANGE_RIGHT
- STOPPING
- YIELDING
- U_TURN
- ...

Input Features for Intent Classification
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The intent classifier examines multiple information sources to infer the driver's goal:

**Vehicle State**:

- Turn signal status (though 30% of drivers don't use them)
- Deceleration profile (different patterns for different maneuvers)
- Steering wheel angle and rate of change
- Lateral position within the lane (drivers often "cheat" toward their intended direction)
- Velocity relative to surrounding traffic

**Map Context**:

- Is the vehicle in a turn-only lane?
- Proximity to intersections, merge points, or exits
- Presence of stop signs, traffic lights, or yield signs
- Legal maneuvers available from current position

.. _l4-Goal_Conditioned_Trajectory_Prediction:

Goal-Conditioned Trajectory Prediction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Core Idea**: First infer the agent's intent (goal), then predict the trajectory to achieve that goal.

**Why Conditioning on Intent Helps**:

- **Without intent**: "Where will this vehicle go?" → highly uncertain, many possibilities
- **With intent**: "Where will this vehicle go given it intends to turn left?" → much more constrained

Mathematically, we transform:

.. math::

   P(\text{trajectory} \mid \text{history}, \text{map}) \rightarrow P(\text{trajectory} \mid \text{history}, \text{map}, \text{intent})

Conditioning on inferred intent produces more accurate and stable predictions
by dramatically reducing the solution space.

**Two-Stage Implementation**:

**Stage 1 - Intent Classification**:

- Network predicts discrete intents: {left turn, straight, right turn, stop, ...}
- Outputs probability distribution: e.g., P(left) = 0.6, P(straight) = 0.3, P(right) = 0.1

**Stage 2 - Conditioned Trajectory Generation**:

- For each high-probability intent, generate K refined trajectories
- Weight final predictions by intent probability
- Result: Multimodal prediction where each mode corresponds to a plausible intent

Real-World Challenges and Limitations
--------------------------------------

The core challenge in prediction is handling rare but critical
scenarios: pedestrians hidden behind buses, aggressive drivers
running red lights, emergency vehicles, objects rolling into the
street. No training dataset can cover all possibilities.

**Implication**: Prediction systems must be robust to out-of-distribution scenarios and should gracefully degrade by increasing uncertainty estimates when encountering unfamiliar situations, rather than failing silently.

Computational Constraints
^^^^^^^^^^^^^^^^^^^^^^^^^^

Prediction must run in real-time on embedded automotive hardware:

- Must process 20-100+ agents simultaneously
- Each agent may require K=6-10 predicted modes
- Update frequency of 10-50 Hz
- Latency budget often <50ms for entire perception-prediction-planning pipeline
- Power and thermal constraints in vehicle compute platforms

This necessitates careful engineering: model quantization, efficient architectures, and intelligent agent prioritization (dedicating more compute to closer, higher-risk agents).

Sensor Limitations and Occlusions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Predictions can only be as good as the underlying perception:

- Occluded agents cannot be tracked or predicted (hidden pedestrian behind bus)
- Sensor noise introduces uncertainty in initial state estimates
- Weather degradation (rain, fog, snow) reduces perception range and accuracy
- Partial observations (seeing only front of vehicle, not turn signals)

**Solution**: Prediction systems should incorporate epistemic uncertainty estimates that increase when input quality is degraded, signaling to the planner to be more conservative.


.. _l4-Evaluating_Prediction_Systems:

Evaluating Prediction Systems
------------------------------

How do we measure whether a prediction system is "good"? This is surprisingly
difficult because we only observe one actual future, but the prediction system
outputs a distribution over many possible futures.

Common Metrics
^^^^^^^^^^^^^^

**Average Displacement Error (ADE)**

Average Euclidean distance between predicted and ground truth positions across the entire prediction horizon. Simple but doesn't capture multimodality.

**Final Displacement Error (FDE)**

Euclidean distance between predicted and ground truth positions at the final time step only. Emphasizes long-term accuracy.

**Miss Rate (MR)**

Percentage of predictions where all predicted modes are farther than a threshold (e.g., 2m) from ground truth. Better captures multimodal performance.

**Negative Log-Likelihood (NLL)**

Measures how well the predicted probability distribution assigns likelihood to the observed outcome. Properly rewards accurate uncertainty estimates.



.. _l4-Challenges_in_Evaluation:

Challenges in Evaluation
^^^^^^^^^^^^^^^^^^^^^^^^^

Standard metrics have significant limitations:

- Ground truth shows only one realized future, but other predicted modes may have been equally valid.
- Metrics don't directly measure what matters: does the prediction enable safe planning?
- Rare but critical events (near-collisions) are underrepresented in datasets.
- Evaluation datasets may not match deployment distribution (domain shift).

The gold standard is **closed-loop evaluation**: running the full AV stack (prediction + planning + control) in simulation or on a test track and measuring overall driving performance (safety, comfort, progress). This tests whether prediction errors actually matter for the driving task.

Summary
-------

Trajectory prediction bridges perception and planning in autonomous vehicles.

- **Time horizons**: 0-2s (physics models), 3-6s (learning models), 6-15s (intent-based).
- **Physics models**: Fast baselines (CV, CA, CTRV, Bicycle) but limited for complex behavior.
- **Learning models**: State-of-the-art (RNNs, GNNs, Transformers) capture context-aware behavior.
- **Multimodality**: Essential for safety—represent multiple futures with probabilities.
- **Evaluation**: ADE, FDE, Miss Rate, NLL measure accuracy and uncertainty calibration.
- **Challenges**: Long-tail events, computational limits, sensor noise, interactive effects.

Active research areas: interactive prediction, uncertainty quantification, long-horizon forecasting, generalization to rare scenarios.

Exercise: Intent Inference Challenge
-------------------------------------

Overview
^^^^^^^^

**Objective**: Experience the challenge of predicting driver intent from noisy observations.

**Scenario**: You are building a prediction system for an automated vehicle. Your task is to predict what a vehicle approaching a 4-way intersection will do based on 3 seconds of observed behavior.

**Possible Intents**:

- LEFT_TURN: Vehicle will turn left
- RIGHT_TURN: Vehicle will turn right
- STRAIGHT: Vehicle continues straight
- U_TURN: Vehicle makes a U-turn

**Files Provided**:

- ``intent_training_features.csv`` - 50 labeled trajectories for analysis
- ``intent_test_features.csv`` - 20 unlabeled trajectories to predict
- ``exercise_worksheet.md`` - Detailed instructions (optional reference)

Available Features
^^^^^^^^^^^^^^^^^^

Each trajectory includes features extracted from 3-second observations:

- **Velocity**: Initial and final speed (m/s)
- **Deceleration**: Average and maximum deceleration (m/s²)
- **Lateral position**: Average and final position in lane (m)
  
  - Negative = left side of lane, Positive = right side

- **Heading change**: Total and final 1-second heading change (degrees)
  
  - Negative = turning left, Positive = turning right

- **Turn signal**: Whether used, which type (LEFT/RIGHT/NONE), activation timing

.. warning::
   **Realistic Challenges Built In**:
   
   - 40% of vehicles don't use turn signals
   - 35% of straight-going vehicles slow down (mimicking turn behavior)
   - Sensor noise and driver inconsistency throughout

Task 1: Pattern Analysis (10 minutes)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Goal**: Identify features that correlate with each intent.

**Instructions**:

- Open ``intent_training_features.csv`` in Excel/Google Sheets/Python
- For each intent type (LEFT_TURN, RIGHT_TURN, STRAIGHT, U_TURN):
  
  - Calculate average deceleration
  - Calculate average lateral position
  - Count turn signal usage percentage
  - Note typical heading change in final 1 second

- Record patterns you observe - which features clearly separate intents?

**Key Questions to Explore**:

- Which features have the clearest separation between intent types?
- Which features are unreliable or noisy?
- Can you identify combinations of features that work better together?

Task 2: Classifier Design (8 minutes)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Goal**: Create a rule-based classifier using IF/THEN logic.

**Instructions**:

- Write decision rules based on patterns you discovered
- Use IF/ELIF/ELSE structure (can be pseudocode or Excel formulas)
- Handle cases where turn signal is missing
- Consider combining multiple features

**Example Starter Rules**:

.. code-block:: python

   IF final_1s_heading_change < -5 degrees THEN predict LEFT_TURN
   ELIF final_1s_heading_change > 5 degrees THEN predict RIGHT_TURN
   ELIF abs(avg_deceleration) < 0.4 THEN predict STRAIGHT
   ELSE ...

**Tips**:

- Start with the most reliable features
- Don't rely solely on turn signals (40% missing!)
- Think about priority ordering when features conflict

Task 3: Make Predictions (4 minutes)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Goal**: Apply your classifier to predict intent for 20 test cases.

**Instructions**:

- Open ``intent_test_features.csv``
- For each of the 20 trajectories, apply your rules
- Record your predicted intent for each trajectory_id
- Methods:
  
  - Manual: Look at each case and apply rules by hand
  - Excel: Use IF formulas to automate predictions
  - Python: Write a simple script

**Submission**: Write down or save your 20 predictions:

.. code-block:: text

   test_000: STRAIGHT
   test_001: LEFT_TURN
   test_002: RIGHT_TURN
   ...

Task 4: Reflection & Discussion (3 minutes)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Reflect on your experience:

- **Which features were most reliable?**
  
  - Most useful: ______________________
  - Least useful: ______________________

- **What was most challenging?**
  
  - Missing turn signals? Overlapping patterns? Conflicting features?

- **How confident are you in your predictions?**
  
  - Very confident / Somewhat confident / Not confident

- **What would you need to improve accuracy to 95%+?**
  
  - More features? Better algorithms? More training data?

Key Takeaways
^^^^^^^^^^^^^

After completing this exercise, you should understand:

- Human behavior is inconsistent - same features can lead to different outcomes
- Turn signals are unreliable - 40% don't use them in our dataset (realistic!)
- Early prediction is extremely difficult - strong signals only appear close to maneuver
- Rule-based systems plateau around 65-75% accuracy - no amount of manual tuning reaches 95%+
- This motivates deep learning approaches:
  
  - Train on millions of examples to learn subtle patterns
  - Temporal models (RNNs, Transformers) handle sequence information
  - Attention mechanisms automatically focus on relevant features
  - Modern systems achieve 85-90% accuracy

**Connection to Course**: Intent inference is easier than full trajectory prediction, but still challenging - motivating the learning-based methods we've discussed today.

Glossary
----------

.. toctree::
   :maxdepth: 2
   :titlesonly:

   glossary

References
----------

.. [1] C. M. Bishop, "Mixture density networks," Technical Report NCRG/94/004, Aston University, 1994.

.. [2] S. Hochreiter and J. Schmidhuber, "Long short-term memory," *Neural Computation*, vol. 9, no. 8, pp. 1735–1780, 1997.

.. [3] A. Graves, "Generating sequences with recurrent neural networks," arXiv preprint arXiv:1308.0850, 2013.

.. [4] K. Cho et al., "Learning phrase representations using rnn encoder-decoder for statistical machine translation," arXiv preprint arXiv:1406.1078, 2014.

.. [5] A. Alahi, K. Goel, V. Ramanathan, A. Robicquet, L. Fei-Fei, and S. Savarese, "Social lstm: Human trajectory prediction in crowded spaces," in *Proceedings of the IEEE conference on computer vision and pattern recognition*, 2016, pp. 961–971.

.. [6] N. Lee, W. Choi, P. Vernaza, C. B. Choy, P. H. Torr, and M. Chandraker, "Desire: Distant future prediction in dynamic scenes with interacting agents," in *Proceedings of the IEEE conference on computer vision and pattern recognition*, 2017, pp. 336–345.

.. [7] A. Vaswani et al., "Attention is all you need," *Advances in neural information processing systems*, vol. 30, 2017.

.. [8] A. Bhattacharyya, M. Fritz, and B. Schiele, "Accurate and diverse sampling of sequences based on a 'best of many' sample objective," in *Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition*, 2018, pp. 8485–8493.

.. [9] N. Deo and M. M. Trivedi, "Convolutional social pooling for vehicle trajectory prediction," in *Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition Workshops*, 2018, pp. 1468–1476.

.. [10] H. Cui et al., "Multimodal trajectory predictions for autonomous driving using deep convolutional networks," in *2019 International Conference on Robotics and Automation (ICRA)*, IEEE, 2019, pp. 2090–2096.

.. [11] A. Levandowski and C. Urmson, "Tesla's advantage in behaviour prediction for autonomous driving," *Seeking Alpha*, Oct. 2019. [Online]. Available: https://seekingalpha.com/article/4284671-teslas-advantage-in-behaviour-prediction-for-autonomous-driving

.. [12] P. Zhang, W. Ouyang, P. Zhang, J. Xue, and N. Zheng, "Sr-lstm: State refinement for lstm towards pedestrian trajectory prediction," in *Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition*, 2019, pp. 12 085–12 094.

.. [13] Y. Chai, B. Sapp, M. Bansal, and D. Anguelov, "Multipath: Multiple probabilistic anchor trajectory hypotheses for behavior prediction," in *Conference on Robot Learning*, PMLR, 2020, pp. 86–99.

.. [14] J. Gao et al., "Vectornet: Encoding hd maps and agent dynamics from vectorized representation," in *Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition*, 2020, pp. 11 525–11 533.

.. [15] J. Li, F. Yang, M. Tomizuka, and C. Choi, "Evolvegraph: Multi-agent trajectory prediction with dynamic relational reasoning," in *Advances in Neural Information Processing Systems*, vol. 33, 2020, pp. 19 783–19 794.

.. [16] T. Salzmann, B. Ivanovic, P. Chakravarty, and M. Pavone, "Trajectron++: Dynamically-feasible trajectory forecasting with heterogeneous data," in *European Conference on Computer Vision*, Springer, 2020, pp. 683–700.

.. [17] F. Giuliari, I. Hasan, M. Cristani, and F. Galasso, "Transformer networks for trajectory forecasting," in *2020 25th International Conference on Pattern Recognition (ICPR)*, IEEE, 2021, pp. 10 335–10 342.

.. [18] J. Ngiam et al., "Scene transformer: A unified architecture for predicting future trajectories of multiple agents," in *International Conference on Learning Representations*, 2021.

.. [19] B. Wilson et al., "Argoverse 2: Next generation datasets for self-driving perception and forecasting," in *Proceedings of the Neural Information Processing Systems Track on Datasets and Benchmarks (NeurIPS Datasets and Benchmarks 2021)*, Argo AI, 2021.

.. [20] D. Bogdoll, M. Nekrasov, T. Joseph, and J. M. Zöllner, "Self-evaluation of automated vehicles based on physics, state-of-the-art motion prediction and user experience," *Scientific Reports*, vol. 13, no. 1, p. 12 899, Aug. 2023. DOI: 10.1038/s41598-023-39811-1

.. [21] N. Nayakanti, R. Al-Rfou, A. Zhou, K. Goel, K. S. Refaat, and B. Sapp, "Wayformer: Motion forecasting via simple & efficient attention networks," in *2023 IEEE International Conference on Robotics and Automation (ICRA)*, IEEE, 2023, pp. 2980–2987.

.. [22] S.-U. Consortium, "Prediction horizon requirements for automated driving: Optimizing safety, comfort, and efficiency," arXiv preprint arXiv:2402.03893v2, Apr. 2024. [Online]. Available: https://arxiv.org/html/2402.03893v2

.. [23] S. Manchingal et al., "Uncertainty-aware autonomous vehicles: Predicting the road ahead," arXiv preprint arXiv:2510.22680, Oct. 2024. [Online]. Available: https://arxiv.org/html/2510.22680

.. [24] W. Zhang et al., "Vehicle trajectory prediction based on attention optimized with real-scene sampling," *Transportation Research*, 2024. [Online]. Available: https://www.tandfonline.com/doi/full/10.1080/21642583.2024.2347889

.. [25] Various Authors, "Motion forecasting for autonomous vehicles: A survey," arXiv preprint arXiv:2502.08664v1, Feb. 2025. [Online]. Available: https://arxiv.org/html/2502.08664v1

.. [26] Various Authors, "Trajectory prediction for autonomous driving: Progress, limitations, and future directions," arXiv preprint arXiv:2503.03262v1, Mar. 2025. [Online]. Available: https://arxiv.org/html/2503.03262v1

.. [27] Waymo LLC, *Waymo open dataset challenges 2025*, 2025. [Online]. Available: https://waymo.com/open/challenges/

.. [28] Y. Yang, N. Negash, and J. Yang, "Influence of prediction horizon on trajectory optimization for autonomous vehicle maneuvers," SAE International, Tech. Rep. 2025-01-8309, Apr. 2025. DOI: 10.4271/2025-01-8309

Next Class
----------

- L5: Behavior planning.
- Live demo: Assignment 2.