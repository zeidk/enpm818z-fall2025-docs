Glossary
========

.. glossary::

   Aleatoric Uncertainty
     Inherent randomness in human behavior that cannot be reduced even with perfect information; grows with prediction horizon.

     See also: :ref:`Quantifying Uncertainty <l4-Quantifying_Uncertainty>`.

   Anchor-Based Approach
     Prediction method using pre-defined template trajectories representing common maneuvers, which are then refined for specific scenarios.

     See also: :ref:`Technical Approaches to Multimodal Prediction <l4-Technical_Approaches_to_Multimodal_Prediction>`.

   Attention Mechanism
     Neural network component that dynamically learns to weight which input elements are most relevant for a given task.

     See also: :ref:`Transformers <l4-transformers>`.

   Average Displacement Error (ADE)
     Evaluation metric measuring the average Euclidean distance between predicted and ground truth positions across the entire prediction horizon.

     See also: :ref:`Evaluating Prediction Systems <l4-Evaluating_Prediction_Systems>`.

   Closed-Loop Evaluation
     Testing methodology where the full AV stack (prediction + planning + control) is evaluated in simulation or real-world scenarios.

     See also: :ref:`Challenges in Evaluation <l4-Challenges_in_Evaluation>`.

   Constant Turn Rate and Velocity (CTRV) Model
     A physics-based model assuming an agent follows a circular arc at constant speed and turn rate.

     See also: :ref:`Physics-Based Prediction Models <l4-physics_models>`.

   Constant Velocity (CV) Model
     A simple physics-based prediction model assuming an object continues in a straight line at constant velocity.

     See also: :ref:`Physics-Based Prediction Models <l4-physics_models>`.

   CVAE
     Conditional Variational Autoencoder; learns a compressed latent representation of possible behaviors and generates diverse trajectories by sampling from this space.

     See also: :ref:`Conditional Variational Autoencoder <l4-Conditional_Variational_Autoencoder>`.

   Encoder-Decoder Architecture
     Neural network structure where an encoder processes input into a hidden representation and a decoder generates output predictions.

     See also: :ref:`LSTMs and GRUs <l4-LSTMs_and_GRUs>`.

   Epistemic Uncertainty
     Uncertainty due to model limitations or insufficient training data; can be reduced with better models and more data.

     See also: :ref:`Quantifying Uncertainty <l4-Quantifying_Uncertainty>`.

   Final Displacement Error (FDE)
     Evaluation metric measuring the Euclidean distance between predicted and ground truth positions at the final time step only.

     See also: :ref:`Evaluating Prediction Systems <l4-Evaluating_Prediction_Systems>`.

   GNN
     Graph Neural Networks; neural networks that model driving scenes as graphs where agents are nodes and interactions are edges.

     See also: :ref:`Graph Neural Networks <l4-graph_neural_networks>`.

   Goal-Conditioned Prediction
     Prediction approach that first infers an agent's intent/goal, then predicts trajectories conditioned on that goal.

     See also: :ref:`Goal-Conditioned Trajectory Prediction <l4-Goal_Conditioned_Trajectory_Prediction>`.

   Intent Inference
     The process of predicting an agent's high-level goal or maneuver (e.g., turn left, go straight, lane change) before predicting the detailed trajectory.

     See also: :ref:`Intent Inference <l4-intent_inference>`.

   Kinematic Bicycle Model
     A physics-based vehicle model representing a car as a bicycle with front and rear axles, enforcing non-holonomic constraints while remaining computationally efficient.

     See also: :ref:`Kinematic Bicycle Model <l4-bicycle_model>`.

   LSTM
     Long Short-Term Memory networks; a type of RNN with gating mechanisms that can learn long-term temporal dependencies in sequential data.

     See also: :ref:`Learning-Based Prediction Models <l4-learning_models>`.

   Message Passing
     Process in GNNs where information propagates through the graph as nodes aggregate information from neighbors.

     See also: :ref:`Graph Neural Networks <l4-graph_neural_networks>`.

   Mixture Density Network (MDN)
     Neural network that outputs parameters of a probability distribution (typically a mixture of Gaussians) rather than single point predictions.

     See also: :ref:`Mixture Density Network <l4-mixture_modal_networks>`.

   Miss Rate (MR)
     Percentage of predictions where all predicted modes are farther than a threshold from ground truth; better captures multimodal performance.

     See also: :ref:`Evaluating Prediction Systems <l4-Evaluating_Prediction_Systems>`.

   Mode
     A distinct possible future behavior or trajectory representing a specific intent or maneuver.

     See also: :ref:`Representing Multimodality <l4-Representing_Multimodality>`.

   Multimodal Prediction
     Prediction approach that outputs multiple distinct trajectory hypotheses (modes), each with an associated probability, representing different possible future behaviors.

     See also: :ref:`Uncertainty and Multimodal Prediction <l4-uncertainty_multimodal_prediction>`.

   Non-Holonomic Constraints
     Movement constraints where a vehicle cannot move arbitrarily in any direction (e.g., cars cannot move sideways).

     See also: :ref:`Kinematic Bicycle Model <l4-bicycle_model>`.

   Prediction Horizon
     The length of time into the future for which predictions are made (e.g., 3 seconds, 6 seconds).

     See also: :ref:`Prediction Time Horizons <l4-Prediction_Time_Horizons>`.

   Social Graph
     A graph representation of a driving scene where agents are nodes and their interactions are edges, used in GNN architectures.

     See also: :ref:`Graph Neural Networks <l4-graph_neural_networks>`.

   Transformer
     Neural network architecture using self-attention mechanisms to dynamically learn which scene elements most influence an agent's future behavior.

     See also: :ref:`Transformers <l4-transformers>`.

   Trajectory Prediction
     The task of forecasting the future movement of dynamic agents (vehicles, pedestrians, cyclists) in autonomous driving scenarios.

     See also: :ref:`Trajectory Prediction Overview <l4-trajectory-prediction-overview>`.
