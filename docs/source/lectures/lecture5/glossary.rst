Glossary
========

.. glossary::

   Action Space
     The set of available decisions an agent can make in an MDP, such as accelerate, brake, change lane left, or maintain lane.

     See also: :ref:`The MDP Tuple <l5-mdp-tuple>`.

   Action-Value Function (:math:`Q`-function)
     A function :math:`Q(s, a)` measuring how good it is to take action :math:`a` in state :math:`s`, then follow a given policy. Used to compare and select actions.

     See also: :ref:`Value Functions <l5-value-functions>`.

   Behavior Tree (BT)
     A modular, hierarchical decision structure using composite nodes (Sequence, Selector) and leaf nodes (Conditions, Actions). Originally developed for video game AI, now widely used in robotics and autonomous vehicles.

     See also: :ref:`Behavior Trees <l5-behavior-trees>`.

   Behavioral Cloning
     A form of imitation learning where a policy is trained via supervised learning to mimic expert demonstrations, mapping observations directly to actions.

     See also: :ref:`Imitation Learning <l5-imitation-learning>`.

   Behavioral Planning
     The tactical level of planning (3–10 second horizon) that decides *what* high-level action the vehicle should take, such as lane change, yield, or stop. Sits between prediction and trajectory planning in the AV stack.

     See also: :ref:`Behavioral Planning Overview <l5-behavioral-planning-overview>`.

   Belief State
     A probability distribution over possible states, used in POMDPs when the true state cannot be directly observed. Updated as new observations are received.

     See also: :ref:`Partial Observability <l5-pomdps>`.

   Bellman Equation
     A recursive equation expressing the value of a state as the immediate reward plus the discounted expected value of the next state. Foundation for dynamic programming algorithms like Value Iteration and Q-Learning.

     See also: :ref:`The Bellman Equation <l5-bellman-equation>`.

   Characteristic Function
     In cooperative game theory, a function :math:`v(S)` that defines the total value that coalition :math:`S` can achieve together.

     See also: :ref:`Shapley Value <l5-shapley-value>`.

   Composite Node
     A control flow node in a behavior tree that determines how child nodes are executed. Includes Sequence (AND logic) and Selector (OR logic) nodes.

     See also: :ref:`Behavior Trees <l5-behavior-trees>`.

   Condition Node
     A leaf node in a behavior tree that checks a state condition (e.g., "Is gap ≥ 3s?") and returns Success or Failure.

     See also: :ref:`Behavior Trees <l5-behavior-trees>`.

   Cooperative Game Theory
     Branch of game theory where players can form coalitions and make binding agreements to achieve shared goals. Uses concepts like Shapley Value for fair allocation.

     See also: :ref:`Game-Theoretic Approaches <l5-game-theoretic-approaches>`.

   Covariate Shift
     The problem in imitation learning where errors compound because the learned policy visits states not seen in the expert's training data, leading to cascading failures.

     See also: :ref:`The Covariate Shift Problem <l5-covariate-shift>`.

   DAgger (Dataset Aggregation)
     An interactive imitation learning algorithm that iteratively collects expert labels for states the learned policy actually visits, addressing covariate shift.

     See also: :ref:`The Covariate Shift Problem <l5-covariate-shift>`.

   Defense in Depth
     A system architecture using multiple safety layers: nominal learned controllers, safety monitors, and fallback controllers, ensuring no single failure leads to an accident.

     See also: :ref:`Hybrid Approaches <l5-hybrid-approaches>`.

   Discount Factor (:math:`\gamma`)
     A parameter between 0 and 1 that determines how much future rewards are valued relative to immediate rewards. :math:`\gamma = 0` is myopic; :math:`\gamma \to 1` is far-sighted.

     See also: :ref:`Return and Discounting <l5-return-and-discounting>`.

   Finite State Machine (FSM)
     A model representing behavior as a set of discrete states with explicit transition rules. The system is in exactly one state at a time, and transitions occur when conditions are met.

     See also: :ref:`Finite State Machines <l5-finite-state-machines>`.

   Hybrid Approach
     A system architecture combining learned components (e.g., perception, prediction, cost functions) with classical structure (e.g., MDP planners, safety monitors, fallback policies).

     See also: :ref:`Hybrid Approaches <l5-hybrid-approaches>`.

   Imitation Learning
     A machine learning approach where an agent learns to perform a task by observing expert demonstrations, rather than through explicit reward functions.

     See also: :ref:`Imitation Learning <l5-imitation-learning>`.

   Leaf Node
     A terminal node in a behavior tree that performs actual work: either checking a Condition or executing an Action.

     See also: :ref:`Behavior Trees <l5-behavior-trees>`.

   Marginal Contribution
     In cooperative game theory, the additional value a player brings when joining a coalition: :math:`\Delta_i(S) = v(S \cup \{i\}) - v(S)`.

     See also: :ref:`Shapley Value <l5-shapley-value>`.

   Markov Decision Process (MDP)
     A mathematical framework for sequential decision-making under uncertainty, defined by the tuple :math:`\langle S, A, P, R, \gamma \rangle`: states, actions, transition probabilities, rewards, and discount factor.

     See also: :ref:`The MDP Tuple <l5-mdp-tuple>`.

   Markov Property
     The assumption that the future depends only on the present state, not on the history of how we got there. Foundation of MDPs.

     See also: :ref:`The Markov Property <l5-markov-property>`.

   Nash Equilibrium
     A strategy profile in non-cooperative game theory where no player can improve their outcome by unilaterally changing their strategy. Models stable interaction patterns.

     See also: :ref:`Nash Equilibrium <l5-nash-equilibrium>`.

   Non-Cooperative Game Theory
     Branch of game theory where players act independently to maximize their own utility without binding agreements. Uses concepts like Nash Equilibrium.

     See also: :ref:`Game-Theoretic Approaches <l5-game-theoretic-approaches>`.

   Partially Observable MDP (POMDP)
     An extension of MDPs where the agent cannot directly observe the true state and must maintain a belief distribution over possible states.

     See also: :ref:`Partial Observability <l5-pomdps>`.

   Policy (:math:`\pi`)
     A mapping from states to actions that tells an agent what to do. Can be deterministic (one action per state) or stochastic (probability distribution over actions).

     See also: :ref:`Value Functions <l5-value-functions>`.

   Responsibility-Sensitive Safety (RSS)
     A formal framework developed by Mobileye/Intel that provides mathematical definitions of safe distances and behaviors, guaranteeing the AV cannot be at fault in collisions if RSS rules are followed.

     See also: :ref:`Implementation Considerations <l5-implementation-considerations>`.

   Return (:math:`G`)
     The cumulative discounted reward from a given time step, used to evaluate the long-term quality of decisions.

     See also: :ref:`Return and Discounting <l5-return-and-discounting>`.

   Reward Function :math:`R(s, a)`
     A function defining the immediate feedback for taking action :math:`a` in state :math:`s`. Positive for desirable outcomes (progress), negative for undesirable ones (collisions).

     See also: :ref:`The MDP Tuple <l5-mdp-tuple>`.

   Selector Node
     A composite node in a behavior tree implementing OR logic: tries children in order, succeeds on first child success, fails only if all children fail.

     See also: :ref:`Behavior Trees <l5-behavior-trees>`.

   Sequence Node
     A composite node in a behavior tree implementing AND logic: executes children in order, fails on first child failure, succeeds only if all children succeed.

     See also: :ref:`Behavior Trees <l5-behavior-trees>`.

   Shapley Value
     A solution concept in cooperative game theory that fairly distributes the total value among players based on their average marginal contribution across all possible orderings.

     See also: :ref:`Shapley Value <l5-shapley-value>`.

   Shield Method
     A safety mechanism in reinforcement learning where a verified safety system monitors the learned policy and vetoes or modifies unsafe actions.

     See also: :ref:`Learning-Based Approaches <l5-learning-based-approaches>`.

   State Space
     The set of all possible configurations in an MDP, including vehicle position, velocity, surrounding objects, traffic signals, etc.

     See also: :ref:`The MDP Tuple <l5-mdp-tuple>`.

   State-Value Function (:math:`V`-function)
     A function :math:`V(s)` measuring how good it is to be in state :math:`s` when following a given policy.

     See also: :ref:`Value Functions <l5-value-functions>`.

   Tactical Planning
     The middle level of autonomous vehicle planning (3–10 second horizon) that determines high-level maneuvers. Also called behavioral planning.

     See also: :ref:`Three Levels of Planning <l5-three-levels-of-planning>`.

   Tick
     The periodic execution cycle of a behavior tree, where the tree is traversed and nodes return their status (Success, Failure, Running).

     See also: :ref:`Behavior Trees <l5-behavior-trees>`.

   Transition Model :math:`P(s'|s,a)`
     A probability function defining the likelihood of reaching state :math:`s'` from state :math:`s` when taking action :math:`a`. Captures uncertainty in outcomes.

     See also: :ref:`The MDP Tuple <l5-mdp-tuple>`.

   V2X (Vehicle-to-Everything)
     Communication technology enabling vehicles to exchange information with other vehicles (V2V), infrastructure (V2I), pedestrians (V2P), and networks (V2N).

     See also: :ref:`Future Direction: V2X Communication <l5-future-directions>`.