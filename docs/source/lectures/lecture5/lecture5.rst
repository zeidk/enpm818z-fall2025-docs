.. _l5-behavioral-planning-overview:

L5 - Behavioral Planning
==================================================

This site accompanies *ENPM818Z - Lecture 5: Behavioral Planning for Automated Vehicles*.

.. admonition:: Resources
   :class: resources

   🔗 `Lecture 5 slides (v1.0) <https://drive.google.com/file/d/1dbuDpxSICYL_kxsXhjzQL5haYFXt8Ryk/view?usp=sharing>`_

   🔗 `FSM Demo (Colab) <https://colab.research.google.com/drive/11IKJ8QBVSaYwVu-q9dflb8jr6bcKkdWZ?usp=sharing>`_

   🔗 `Behavior Tree Demo <https://drive.google.com/file/d/1DbcRgau5MAwdN0oVBnxIze9uzn627LsH/view?usp=sharing>`_

Overview
--------

**Bridge: From Prediction to Action**

Behavioral planning sits between **Prediction** and **Trajectory Planning** in the AV stack.
It answers the critical question:

   *"Given what we perceive and predict, what high-level action should the vehicle take?"*

The outputs are *intentions*, not trajectories: change lanes or stay, yield or proceed
at an intersection, merge now or wait for a larger gap, slow for a pedestrian or
maintain speed. The **motion planner** then converts these intentions into precise,
drivable paths.

Learning Objectives
-------------------

1. **Classical Approaches**: Understand Finite State Machines and Behavior Trees for rule-based planning
2. **Decision-Making Under Uncertainty**: Apply Markov Decision Processes (MDPs) and understand partial observability
3. **Interactive & Learning-Based Methods**: Explore game theory fundamentals and imitation learning basics
4. **Implementation Considerations**: Understand system integration, safety verification, and evaluation metrics

.. important::
   **Goal**: Understand the spectrum of approaches (from interpretable rules to learned
   policies) and their tradeoffs.


.. _l5-three-levels-of-planning:

Three Levels of Planning
------------------------

Planning in autonomous vehicles operates across three distinct levels:

.. list-table::
   :widths: 25 20 55
   :header-rows: 1

   * - **Level**
     - **Horizon**
     - **Example**
   * - Strategic (Mission)
     - Minutes–Trip
     - "Take I-95 North for 47 miles"
   * - **Tactical (Behavior)**
     - **3–10 sec**
     - **"Change to left lane now" or "Yield to oncoming traffic"**
   * - Operational (Trajectory)
     - 0–2 sec
     - Smooth, collision-free path with precise position and velocity

This lecture focuses on the **Tactical** level: deciding *what* to do, not *how* to execute it.


.. _l5-why-is-it-hard:

Why Is Behavioral Planning Hard?
--------------------------------

**1. Uncertainty Everywhere**

Perception and prediction are never fully certain. The planner must act despite
incomplete confidence.

**2. Multi-Agent Interaction**

Driving is *social*. Our actions influence others, and their responses influence us.
Lane changes, merges, and crossings require implicit negotiation.

**3. Competing Objectives**

- Safety vs. efficiency
- Traffic rules vs. social norms
- Comfort vs. progress

**4. Long-Tail Edge Cases**

Construction zones, emergency vehicles, jaywalkers, debris: critical scenarios
that must be handled safely.


.. _l5-classical-approaches:

Classical Approaches
--------------------

We begin with **rule-based** methods that have been deployed in real autonomous vehicle
systems. These approaches are interpretable, verifiable, and form the foundation of
many production planners.


.. _l5-finite-state-machines:

Finite State Machines (FSM)
^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Core Idea**: Model driving behavior as a set of **discrete states** with well-defined
**transition rules**.

**Properties**:

- System is in *exactly one state* at a time
- Transitions occur when conditions are met
- States represent high-level behaviors

**Example States**:

- Lane Keeping
- Prepare Lane Change
- Execute Lane Change
- Emergency Stop

**Transition Logic Example: Lane Change**

Transition from *Prepare Lane Change* → *Execute Lane Change* requires:

- Sufficient gap in target lane (≥ 3 seconds headway)
- Blind spot confirmed clear
- Motion planner returns feasible trajectory

*Abort conditions* (active during execution):

- Sudden braking detected in target lane
- Vehicle accelerating into the gap
- Trajectory no longer feasible

.. note::
   Every transition encodes domain knowledge: the "rules of the road" made explicit.

**Strengths**:

- **Interpretable**: Every decision is traceable to specific conditions
- **Verifiable**: Formal methods can prove safety properties
- **Debuggable**: "Why did it brake?" has a clear answer
- **Deployed**: Used in Autoware, Apollo, and many production systems

**Limitations**:

- **State explosion**: Real driving requires many states and transitions
- **Brittle to edge cases**: Every scenario must be anticipated
- **No uncertainty reasoning**: Conditions are binary (true/false)
- **Poor multi-agent handling**: Hard to encode negotiation


.. _l5-behavior-trees:

Behavior Trees (BT)
^^^^^^^^^^^^^^^^^^^

**Origin and Motivation**: Developed for video game AI, now widely used in robotics.
Addresses FSM scalability through **modularity** and **reuse**.

**Key Differences from FSMs**:

- **Hierarchical**: Behaviors compose into larger behaviors
- **Reusable**: "Safe lane change" subtree works in multiple contexts
- **Reactive**: Tree is "ticked" each cycle; responds to changing conditions

**Execution Model**: Each node returns one of three statuses:

- **Success** — Task completed
- **Failure** — Task could not be completed
- **Running** — Task still in progress

Node Types
""""""""""

**Composite Nodes (Control Flow)**:

**Sequence (→) — AND logic**:

- Executes children in order
- Fails on first child failure
- Succeeds only if *all* succeed
- *Example*: Parallel parking steps—each must succeed

**Selector (?) — OR logic**:

- Tries children in order
- Succeeds on first child success
- Fails only if *all* fail
- *Example*: Try parallel parking; if unavailable, try garage

**Leaf Nodes**:

- **Condition**: Check a state (e.g., "Is gap ≥ 3s?") → Success/Failure
- **Action**: Execute behavior (e.g., "Activate turn signal") → Success/Failure/Running

Example: Lane Change with Fallback
""""""""""""""""""""""""""""""""""

A behavior tree for lane change might have:

- **Root Selector (?)**: "Try to change lanes. If that fails, fall back to maintaining lane"
- **Sequence Node (→)**: Lane change with four steps (Gap? → Blind Spot? → Signal → Execute)
- **Fallback Action**: Maintain Lane

**Execution Scenarios**:

*Scenario A: Successful Lane Change*

- Tick 1: Gap ✓ → Blind Spot ✓ → Signal ✓ → Execute (Running...)
- Tick 2: Gap ✓ → Blind Spot ✓ → Signal ✓ → Execute (Running...)
- Tick 3: Gap ✓ → Blind Spot ✓ → Signal ✓ → Execute ✓ (Success!)

*Scenario B: No Gap Available*

- Tick 1: Gap ✗ → Sequence fails → Selector tries fallback → Maintain Lane ✓

*Scenario C: Gap Closes Mid-Maneuver*

- Tick 1: Gap ✓ → Blind Spot ✓ → Signal ✓ → Execute (Running...)
- Tick 2: Gap ✗ (other car accelerated!) → Sequence fails → Maintain Lane ✓

**Strengths**:

- **Modular**: Subtrees are reusable across scenarios
- **Readable**: Clear success/failure semantics
- **Scalable**: Easier to extend than flat FSMs
- **Reactive**: Handles dynamic conditions via periodic ticking

**Limitations**:

- **Still hand-crafted**: Requires manual design of all branches
- **Ordering matters**: Selector/sequence priority affects behavior
- **No uncertainty**: Conditions remain binary
- **Can grow complex**: Large trees become hard to maintain

.. note::
   BTs excel at structured tasks: parking, lane assistance, tactical maneuvers with
   predictable steps.


Classical Methods: When to Use What
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 25 37 38
   :header-rows: 1

   * - **Aspect**
     - **Finite State Machine**
     - **Behavior Tree**
   * - Structure
     - Flat states + transitions
     - Hierarchical tree
   * - Reusability
     - Low (copy-paste states)
     - High (subtrees)
   * - Scalability
     - Poor (state explosion)
     - Better (modular)
   * - Debugging
     - Trace transitions
     - Trace tree path
   * - Best For
     - Simple, well-defined scenarios
     - Complex tasks with fallbacks

Both methods are **deterministic** and assume perfect state observation.
Next: handling **uncertainty**.


.. _l5-decision-making-under-uncertainty:

Decision-Making Under Uncertainty
---------------------------------

Classical methods assume a **predictable, fully observable** world. Real driving
violates both assumptions.

**The Problem with Binary Conditions**:

- "Is pedestrian crossing?" → Yes/No
- Sensor noise causes oscillation
- No reasoning about *risk*

**What we need**: A framework that reasons about probabilities, uncertain outcomes,
and long-term consequences.

⇒ **Markov Decision Processes (MDPs)**


.. _l5-markov-property:

The Markov Property
^^^^^^^^^^^^^^^^^^^

**Core Assumption**: The future depends **only on the present state**, not on the
history of how we got there.

.. math::

   P(S_{t+1} \mid S_t, S_{t-1}, \ldots, S_0) = P(S_{t+1} \mid S_t)

**Driving Interpretation**: To decide whether to brake *now*, I need to know my
current speed and position, distance to the car ahead, and current traffic light state.
I **don't** need to know what my speed was 10 seconds ago.

The state encodes everything relevant for decision-making.


.. _l5-mdp-tuple:

The MDP Tuple
^^^^^^^^^^^^^

An MDP is defined by five components:

.. math::

   \mathcal{M} = \langle S, A, P, R, \gamma \rangle

- **S — State Space**: All possible configurations (position, velocity, traffic light, surrounding vehicles)
- **A — Action Space**: Available decisions (accelerate, brake, change lane left, maintain lane)
- **P(s' | s, a) — Transition Model**: Probability of reaching s' from s via action a (captures uncertainty in outcomes)
- **R(s, a) — Reward Function**: Immediate feedback for taking action a in state s (positive for progress, negative for collisions)
- **γ ∈ [0,1] — Discount Factor**: Tradeoff between immediate and future rewards


.. _l5-return-and-discounting:

Return and Discounting
^^^^^^^^^^^^^^^^^^^^^^

The agent maximizes **total expected reward over time**, not just immediate reward:

.. math::

   G_t = R_{t+1} + \gamma R_{t+2} + \gamma^2 R_{t+3} + \cdots = \sum_{k=0}^{\infty} \gamma^k R_{t+k+1}

**The Role of γ**:

- γ = 0: **Myopic** — Only care about next reward
- γ → 1: **Far-sighted** — Future matters almost as much as now

For autonomous vehicles, typically γ ≈ 0.95–0.99. Safety matters long-term, but
immediate collisions matter most.


.. _l5-value-functions:

Value Functions
^^^^^^^^^^^^^^^

**Policy π**: A mapping from states to actions. Tells the agent *what to do*.

.. math::

   \pi(a \mid s) = P(A_t = a \mid S_t = s)

**State-Value Function V^π(s)**: "How good is it to **be in state** s and follow policy π?"

.. math::

   V^\pi(s) = \mathbb{E}_\pi[G_t \mid S_t = s]

**Action-Value Function Q^π(s, a)**: "How good is it to **take action** a in state s, then follow π?"

.. math::

   Q^\pi(s, a) = \mathbb{E}_\pi[G_t \mid S_t = s, A_t = a]

Value functions let us compare actions: choose the one with highest Q(s,a).


.. _l5-bellman-equation:

The Bellman Equation
^^^^^^^^^^^^^^^^^^^^

**Core Idea**: The value of a state is not just about what you get immediately.
It also depends on what future situations that state leads to, and how good
those future situations are.

**Bellman Expectation Equation**:

.. math::

   V^\pi(s) = \sum_{a \in A} \pi(a \mid s) \left[ \underbrace{R(s,a)}_{\text{Reward now}} + \gamma \underbrace{\sum_{s' \in S} P(s' \mid s, a) V^\pi(s')}_{\text{Expected future value}} \right]

Components:

- π(a | s) — **Policy**: how likely the agent is to choose action a in state s
- R(s,a) — **Immediate reward**: what the agent gets right now
- γ — **Discount factor**: how much future rewards matter
- P(s' | s, a) — **Transition probability**: how likely we end up in next state s'
- V^π(s') — **Future value**: how good the next state is

**Why This Matters**:

- Turns decision making into a **recursive scoring problem**
- Forms the basis of **Value Iteration**, **Policy Iteration**, and **Q-Learning**
- Enables **dynamic programming**: solve complex planning by breaking it into smaller decisions


.. _l5-mdp-example:

Example: Highway Lane Change
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Scenario**: We are stuck behind a slow truck. Should we change lanes to pass?

**States S**:

- s_behind: Behind truck
- s_free: Free-flow traffic
- s_danger: Unsafe situation

**Actions A**:

- a_stay: Maintain lane
- a_change: Change lane

**Dynamics & Rewards**:

- **If stay**: 100% remain in s_behind, Reward: -2 (frustration)
- **If change**: 85% → s_free (reward +10), 15% → s_danger (reward -100)

**Discount**: γ = 0.95

**Computing Action Values**:

*Option 1: Stay Behind Truck* — Receive -2 forever (geometric series):

.. math::

   Q(s_{\text{behind}}, a_{\text{stay}}) = \frac{-2}{1 - 0.95} = \mathbf{-40}

*Option 2: Change Lanes* — Assume V(s_free)=+80 and V(s_danger)=-40:

.. math::

   Q(s_{\text{behind}}, a_{\text{change}}) = 0.85[10 + 0.95(80)] + 0.15[-100 + 0.95(-40)] = \mathbf{+52.4}

**The Optimal Decision**: Despite 15% risk of a bad outcome, **changing lanes** is optimal because:

- The long-term cost of staying (-40) exceeds the expected cost of the risk
- Success leads to sustained positive value (+80 in free-flow)
- The discount factor makes prolonged frustration expensive


.. _l5-pomdps:

Partial Observability (POMDPs)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**The MDP Assumption**: The agent perfectly observes the current state s_t.

**The Driving Reality**: Sensors have noise, and critical information is often **hidden**:

*What We Can Observe*:

- Position and velocity
- Lane markings
- Traffic light color
- Vehicle bounding boxes

*What Remains Hidden*:

- **Intentions**: Lane change or just drifting?
- **Attention**: Has the driver seen us?
- **Occlusions**: Child behind parked truck?
- **State of mind**: Aggressive or cautious?

**The Belief State Concept**: Since we don't know the true state, we maintain a
**belief**—a probability distribution over possible states.

*Example: Ambiguous Turn Signal*

A car in the right lane has its left blinker on:

- 60%: Intends to change lanes (cut-in)
- 30%: Forgot to turn off signal
- 10%: Planning to turn at intersection

As we observe more (speed change, lateral movement), we **update our belief**.

**POMDP Planning**: Instead of State → Action, we use Belief → Action.

.. note::
   POMDPs are computationally expensive but their **principles**—maintaining beliefs,
   reasoning under uncertainty—are fundamental to modern AV planners.


MDP Summary
^^^^^^^^^^^

**What We Covered**:

1. **Markov Property**: Future depends only on present state
2. **MDP Tuple**: ⟨S, A, P, R, γ⟩ formalizes the decision problem
3. **Return & Discounting**: Maximize cumulative reward; γ balances now vs. later
4. **Value Functions**: V(s) and Q(s,a) measure long-term quality
5. **Bellman Equation**: Recursive decomposition enables efficient algorithms

**Limitations**:

- **State explosion**: Continuous driving states are infinite
- **Model required**: Need accurate P and R (hard to specify)
- **Single agent**: Doesn't model other drivers as strategic actors


.. _l5-game-theoretic-approaches:

Game-Theoretic Approaches
-------------------------

So far, we have treated other agents as **predictable entities**. But driving is
fundamentally **interactive**.

- Our actions influence what other drivers do
- Their responses influence what we should do
- Their anticipation of our response influences them...

⇒ **Game theory** provides tools for reasoning about strategic interactions.


Two Types of Games
^^^^^^^^^^^^^^^^^^

**Non-Cooperative Games**

Players act **independently** to maximize their own utility. No binding agreements.

*Key concept: Nash Equilibrium* — A state where no player can improve their outcome
by *unilaterally* changing their strategy.

*AV Applications*: Highway merging, unprotected left turns, intersection negotiation

**Cooperative Games**

Players can form **coalitions** and make binding agreements to achieve shared goals.

*Key concept: Shapley Value* — A fair way to distribute the *total benefit* among
players based on their marginal contributions.

*AV Applications*: V2V platooning, cooperative intersection management, shared autonomy fleets

.. note::
   **When to use which?** Non-cooperative for interactions with unknown human drivers;
   cooperative for V2V-enabled vehicles that can communicate and coordinate.


.. _l5-nash-equilibrium:

Nash Equilibrium: The Merge Game
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Scenario**: Two vehicles approach a highway merge. Each must decide: **Yield** or **Proceed**.

**Payoff Matrix** (V1, V2):

.. list-table::
   :widths: 30 35 35
   :header-rows: 1

   * - 
     - **V2: Yield**
     - **V2: Proceed**
   * - **V1: Yield**
     - (3, 3)
     - (2, 5)
   * - **V1: Proceed**
     - (5, 2)
     - (-10, -10) ← collision

**Analysis**:

- **Two Nash Equilibria**: (Proceed, Yield) → (5, 2) and (Yield, Proceed) → (2, 5)
- Both are stable—no one wants to deviate
- **Problem**: Which equilibrium?
- Real driving uses signals, timing, and social norms to coordinate

**Nash Equilibrium**: No player can improve by changing strategy alone. In driving,
this models stable interaction patterns—but doesn't tell us *which* pattern emerges.


.. _l5-shapley-value:

Shapley Value: Cooperative Platooning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Scenario**: Three vehicles form a **platoon** on the highway, drafting to save fuel.
**Vehicle A** leads (no drafting benefit), **B** and **C** follow and save fuel by drafting.

**Coalition Values** (Total Fuel Savings):

.. list-table::
   :widths: 30 20 50
   :header-rows: 1

   * - **Coalition**
     - **Savings**
     - **Why?**
   * - {A}, {B}, {C} alone
     - $0
     - No one to draft/no leader
   * - {A, B}
     - $10
     - B drafts behind A
   * - {A, C}
     - $10
     - C drafts behind A
   * - {B, C}
     - $0
     - No leader to draft behind!
   * - {A, B, C}
     - $20
     - Full platoon

**Shapley Value Axioms**:

1. **Efficiency**: Total payout equals total value—nothing is wasted
2. **Symmetry**: Players who contribute equally receive equal shares
3. **Null Player**: Players who contribute nothing receive nothing
4. **Additivity**: Value in combined games = sum of values in separate games

**Marginal Contribution**:

The contribution of player i is determined by what is gained by adding them:

.. math::

   \Delta_i(S) = v(S \cup \{i\}) - v(S)

**Final Allocation**:

- φ_A = $10 (50%) — Essential: platoon cannot form without leader
- φ_B = $5 (25%) — Contributes as follower
- φ_C = $5 (25%) — Contributes as follower

The Shapley Value **incentivizes** A to lead even though A gets no direct drafting
benefit—A is compensated for enabling the coalition!


Game Theory Summary
^^^^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 20 40 40
   :header-rows: 1

   * - **Aspect**
     - **Non-Cooperative**
     - **Cooperative**
   * - Key Concept
     - Nash Equilibrium
     - Shapley Value
   * - Assumption
     - Players act selfishly
     - Players can coordinate
   * - Communication
     - None required
     - V2V/V2X enabled
   * - AV Example
     - Merge negotiation with humans
     - Platoon formation, intersection scheduling
   * - Challenge
     - Equilibrium selection
     - Ensuring truthful participation

**Key Insight**: Most current AV interactions with human drivers are *non-cooperative*
(no communication). As V2X technology matures, *cooperative* game theory will enable
more efficient coordination.


.. _l5-learning-based-approaches:

Learning-Based Approaches
-------------------------

The approaches we've discussed—FSMs, MDPs, game theory—rely on **explicit models**
and **hand-crafted components**.

- Can we discover behaviors **directly from data**?
- Handle complexity and edge cases that defy explicit engineering?
- Learn subtle patterns that humans demonstrate but struggle to articulate?

**Three approaches**: *Imitation Learning* · *Deep Reinforcement Learning* · *Hybrid Methods*


.. _l5-imitation-learning:

Imitation Learning
^^^^^^^^^^^^^^^^^^

**The Idea**: Collect expert demonstrations (humans driving), train a model to predict
what action the expert would take in each situation.

**Supervised Learning Formulation**:

*Input (Observations)*: Camera images, bird's-eye view from sensor fusion, feature vectors

*Output (Actions)*: Steering angle, acceleration, lane change decision

**Advantages**:

- No need to manually design reward functions or state representations
- Can learn subtle behaviors difficult to encode as explicit rules
- Leverages vast amounts of human driving data

.. note::
   Companies like Tesla and Wayve use imitation learning as a core component.


.. _l5-covariate-shift:

The Covariate Shift Problem
^^^^^^^^^^^^^^^^^^^^^^^^^^^

**The Critical Challenge**: Training data comes from states visited by the **expert policy**.
But the learned policy makes slightly different decisions, leading to slightly different states...

::

   Small error → New state → No training data → Bigger error → (compounds over time)

**Concrete Example**:

- Expert driver stays **precisely centered** in lane
- Learned policy is almost perfect but occasionally drifts toward edge
- Training data has **no examples** of being off-center
- Policy hasn't learned to correct errors → vehicle drifts off road

The agent finds itself in states unlike anything in training, with no reliable
guidance for recovery.

**DAgger: Solving Covariate Shift**

*Dataset Aggregation through Interactive Learning*:

1. **Initialize**: Train policy π₁ on expert demonstrations D
2. **Deploy**: Run learned policy πᵢ in environment
3. **Label**: Expert provides labels for states the *learned policy* actually visits
4. **Aggregate**: D ← D ∪ D_new
5. **Retrain**: Train π_{i+1} on aggregated dataset
6. **Iterate**: Repeat until policy is robust

**Real-World DAgger for AVs**: Safety drivers ride in autonomous vehicles and occasionally
take over when needed, providing corrections for encountered states.

*Key insight*: Training distribution progressively matches states actually visited
by the learned policy.


Deep Reinforcement Learning
^^^^^^^^^^^^^^^^^^^^^^^^^^^

**The Idea**: Rather than learning from demonstrations, learn through **interaction
with the environment**:

- Agent takes actions, observes rewards
- Updates policy to increase expected cumulative reward
- Deep neural networks enable learning in high-dimensional spaces

**Key Methods**:

- **DQN**: Learns Q-function for discrete actions; uses experience replay and target networks
- **Policy Gradients**: Directly optimize policy parameters to maximize expected reward
- **PPO**: Constrains policy updates to avoid destructive changes; stable training

**The Fundamental Challenge**: Deep RL requires **millions of training steps**. In a
real vehicle, this means millions of miles including inevitable crashes during
exploration—**clearly unacceptable**.

**Safety Solutions**:

- **Reward Shaping**: Add intermediate rewards to guide learning toward safe behaviors
- **Constrained RL**: Explicitly constrain policy to satisfy safety constraints
- **Shield Methods**: Separate verified safety system vetoes or modifies unsafe actions


.. _l5-hybrid-approaches:

Hybrid Approaches
^^^^^^^^^^^^^^^^^

**The Industry Reality**: The most successful deployed systems **don't replace everything
with neural networks**. Instead, they use learning for specific components while
maintaining interpretable structure.

**Powerful Paradigms**:

- **Learned cost functions**: MDP planner with neural network outputting costs
- **Learned heuristics for MCTS**: Replace rollout policies with learned approximations (à la AlphaGo Zero)
- **Learned prediction**: Classical planner uses learned trajectory forecasts

**Defense in Depth**:

- **Nominal controller**: Sophisticated learning handles normal operation
- **Safety monitors**: Watch for constraint violations
- **Fallback controllers**: Provide proven-safe behavior when problems arise

This multi-layered architecture mirrors safety-critical systems in aerospace—multiple
layers reduce the likelihood that any single failure leads to an accident.


Industry Landscape
^^^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 15 85
   :header-rows: 1

   * - **Company**
     - **Approach**
   * - Waymo
     - Classical, structured planning with learned components for prediction and perception; heavy emphasis on verification and safety by design
   * - Tesla
     - Heavy reliance on learning; neural networks extensive in FSD; massive data from fleet
   * - Wayve
     - Explicitly end-to-end learning; demonstrated systems mapping sensors to controls
   * - Aurora
     - Hybrid approach for highway trucking; balances learning and classical methods


Learning Summary
^^^^^^^^^^^^^^^^

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - **Concept**
     - **Description**
   * - Imitation Learning
     - Learn to mimic expert behavior from demonstrations
   * - Covariate Shift
     - Errors compound when policy visits states unseen in training
   * - DAgger
     - Iteratively collect data from states the learned policy actually visits
   * - Deep RL
     - Learn through interaction; requires simulation for sample efficiency
   * - Shield Methods
     - Safety layer vetoes unsafe RL actions
   * - Hybrid Systems
     - Combine learned components with classical safety guarantees

**Learning enables handling complexity; structure ensures safety. Both are needed.**


.. _l5-implementation-considerations:

Implementation Considerations
-----------------------------

Understanding algorithms is one thing; **building a working system** is another.

- How does the behavioral planner integrate with other modules?
- How do we ensure safe operation?
- How do we measure success?


System Integration
^^^^^^^^^^^^^^^^^^

The behavioral planner communicates with upstream and downstream modules:

**Inputs (from Perception & Prediction)**:

- Current vehicle state
- Detected objects and their predicted trajectories
- Map information and route

**Outputs (to Motion Planner)**:

- High-level maneuver command (lane keep, lane change, stop, etc.)
- Target lane or position
- Desired velocity profile


Responsibility-Sensitive Safety (RSS)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Developed by Mobileye/Intel, RSS provides **formal safety guarantees** independent
of the behavioral planner's algorithm:

**Core Principles**:

- Define "safe" longitudinal and lateral distances mathematically
- Assume worst-case behavior from other agents
- Guarantee: if the AV follows RSS rules, it cannot be at fault in a collision

**RSS as a Safety Layer**:

- Behavioral planner proposes actions
- RSS checks if action maintains safe distances
- If unsafe, RSS modifies or vetoes the action

This provides a formal safety guarantee even if the planner uses learned components.


Key Performance Indicators (KPIs)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Safety Metrics**:

- Collision rate
- Near-miss frequency
- Disengagement rate (when human takes over)

**Efficiency Metrics**:

- Average speed vs. speed limit
- Time to complete route
- Traffic flow contribution

**Comfort Metrics**:

- Acceleration/jerk statistics
- Lane-keeping smoothness
- Passenger comfort ratings


Open Challenges
^^^^^^^^^^^^^^^

**Technical Challenges**:

- Handling unknown scenarios not in training/rules
- Scaling verification to complex systems
- Closing the sim-to-real gap

**Social Challenges**:

- Regulatory approval
- Public trust
- Ethical dilemmas (no societal consensus yet)


.. _l5-future-directions:

Future Direction: V2X Communication
-----------------------------------

**The Current Paradigm**: We *infer* other vehicles' intentions from observed trajectories.

**Vehicle-to-Everything (V2X)**: What if vehicles could **communicate directly**?

**Information Exchange**:

- Vehicles broadcast intentions
- Traffic lights broadcast timing
- Construction zones broadcast layout
- Emergency vehicles broadcast route

**Benefits**:

- Reduced uncertainty
- Earlier hazard awareness
- Coordinated behavior
- More efficient intersections

V2X could fundamentally transform behavioral planning—from guessing intentions
to knowing them. Active research and early deployments underway.


Lecture Summary
---------------

**What We Covered**:

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - **Topic**
     - **Key Points**
   * - Classical Approaches
     - FSMs and Behavior Trees: interpretable, verifiable, but brittle
   * - MDPs
     - Formal framework for decision-making under uncertainty
   * - POMDPs
     - Handling hidden state through belief distributions
   * - Game Theory
     - Reasoning about strategic multi-agent interactions
   * - Learning
     - Imitation learning, covariate shift, hybrid systems
   * - Implementation
     - System integration, RSS safety, KPI evaluation

**The Big Picture**: Behavioral planning sits at the intersection of control theory,
machine learning, and social science. No single approach dominates—the best systems
combine multiple methods with layered safety guarantees.


Glossary
--------

.. toctree::
   :maxdepth: 2
   :titlesonly:

   glossary


Resources
---------

**Full AV Stacks**:

- `Autoware <https://github.com/autowarefoundation/autoware>`_ — ROS2-based; behavior velocity planner, lane change modules
- `Apollo (Baidu) <https://github.com/ApolloAuto/apollo>`_ — Scenario-based planning (FSM + optimization)

**Behavior Tree Libraries**:

- `py_trees (Python) <https://github.com/splintered-reality/py_trees>`_ — Excellent docs and ROS integration
- `BehaviorTree.CPP <https://github.com/BehaviorTree/BehaviorTree.CPP>`_ — C++; Groot visualizer

**Simulators**:

- `CARLA <https://github.com/carla-simulator/carla>`_ — Urban driving; supports IL and RL
- `CommonRoad <https://commonroad.in.tum.de>`_ — Benchmark scenarios for planning

**Key Papers**:

- Werling et al. (2010) — Frenet planning
- Codevilla et al. (2018) — Conditional IL
- Colledanchise & Ögren — BT textbook

**Getting Started**:

.. code-block:: bash

   pip install py-trees
   py-trees-demo-selector


Next Class
----------

- L6: Trajectory planning
- Demo: RWA3 starter code