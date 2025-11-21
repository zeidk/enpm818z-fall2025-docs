Glossary
========

.. glossary::

   Trajectory Prediction
     The task of forecasting the future movement of dynamic agents (vehicles, pedestrians, cyclists) in autonomous driving scenarios.

     See also: :doc:`Trajectory Prediction Overview </lecture4/lecture4>`.

   Multimodal Prediction
     Prediction approach that outputs multiple distinct trajectory hypotheses (modes), each with an associated probability, representing different possible future behaviors.

     See also: :doc:`Uncertainty and Multimodal Prediction </lecture4/lecture4>`.

   Intent Inference
     The process of predicting an agent's high-level goal or maneuver (e.g., turn left, go straight, lane change) before predicting the detailed trajectory.

     See also: :doc:`Intent Inference </lecture4/lecture4>`.

   Kinematic Bicycle Model
     A physics-based vehicle model representing a car as a bicycle with front and rear axles, enforcing non-holonomic constraints while remaining computationally efficient.

     See also: :doc:`Kinematic Bicycle Model </lecture4/lecture4>`.

   Constant Velocity (CV) Model
     A simple physics-based prediction model assuming an object continues in a straight line at constant velocity.

     See also: :doc:`Physics-Based Prediction Models </lecture4/lecture4>`.

   Constant Turn Rate and Velocity (CTRV) Model
     A physics-based model assuming an agent follows a circular arc at constant speed and turn rate.

     See also: :doc:`Physics-Based Prediction Models </lecture4/lecture4>`.

   LSTM
     Long Short-Term Memory networks; a type of RNN with gating mechanisms that can learn long-term temporal dependencies in sequential data.

     See also: :doc:`Learning-Based Prediction Models </lecture4/lecture4>`.

   GNN
     Graph Neural Networks; neural networks that model driving scenes as graphs where agents are nodes and interactions are edges.

     See also: :doc:`Graph Neural Networks </lecture4/lecture4>`.

   Transformer
     Neural network architecture using self-attention mechanisms to dynamically learn which scene elements most influence an agent's future behavior.

     See also: :doc:`Transformers </lecture4/lecture4>`.

   Mixture Density Network (MDN)
     Neural network that outputs parameters of a probability distribution (typically a mixture of Gaussians) rather than single point predictions.

     See also: :doc:`Technical Approaches to Multimodal Prediction </lecture4/lecture4>`.

   CVAE
     Conditional Variational Autoencoder; learns a compressed latent representation of possible behaviors and generates diverse trajectories by sampling from this space.

     See also: :doc:`Technical Approaches to Multimodal Prediction </lecture4/lecture4>`.

   Aleatoric Uncertainty
     Inherent randomness in human behavior that cannot be reduced even with perfect information; grows with prediction horizon.

     See also: :doc:`Quantifying Uncertainty </lecture4/lecture4>`.

   Epistemic Uncertainty
     Uncertainty due to model limitations or insufficient training data; can be reduced with better models and more data.

     See also: :doc:`Quantifying Uncertainty </lecture4/lecture4>`.

   Average Displacement Error (ADE)
     Evaluation metric measuring the average Euclidean distance between predicted and ground truth positions across the entire prediction horizon.

     See also: :doc:`Evaluating Prediction Systems </lecture4/lecture4>`.

   Final Displacement Error (FDE)
     Evaluation metric measuring the Euclidean distance between predicted and ground truth positions at the final time step only.

     See also: :doc:`Evaluating Prediction Systems </lecture4/lecture4>`.

   Miss Rate (MR)
     Percentage of predictions where all predicted modes are farther than a threshold from ground truth; better captures multimodal performance.

     See also: :doc:`Evaluating Prediction Systems </lecture4/lecture4>`.

   Prediction Horizon
     The length of time into the future for which predictions are made (e.g., 3 seconds, 6 seconds).

     See also: :doc:`Prediction Time Horizons </lecture4/lecture4>`.

   Mode
     A distinct possible future behavior or trajectory representing a specific intent or maneuver.

     See also: :doc:`Representing Multimodality </lecture4/lecture4>`.

   Non-Holonomic Constraints
     Movement constraints where a vehicle cannot move arbitrarily in any direction (e.g., cars cannot move sideways).

     See also: :doc:`Kinematic Bicycle Model </lecture4/lecture4>`.

   Social Graph
     A graph representation of a driving scene where agents are nodes and their interactions are edges, used in GNN architectures.

     See also: :doc:`Graph Neural Networks </lecture4/lecture4>`.

   Attention Mechanism
     Neural network component that dynamically learns to weight which input elements are most relevant for a given task.

     See also: :doc:`Transformers </lecture4/lecture4>`.

   Goal-Conditioned Prediction
     Prediction approach that first infers an agent's intent/goal, then predicts trajectories conditioned on that goal.

     See also: :doc:`Goal-Conditioned Trajectory Prediction </lecture4/lecture4>`.

   Encoder-Decoder Architecture
     Neural network structure where an encoder processes input into a hidden representation and a decoder generates output predictions.

     See also: :doc:`LSTMs and GRUs </lecture4/lecture4>`.

   Message Passing
     Process in GNNs where information propagates through the graph as nodes aggregate information from neighbors.

     See also: :doc:`Graph Neural Networks </lecture4/lecture4>`.

   Anchor-Based Approach
     Prediction method using pre-defined template trajectories representing common maneuvers, which are then refined for specific scenarios.

     See also: :doc:`Technical Approaches to Multimodal Prediction </lecture4/lecture4>`.

   Closed-Loop Evaluation
     Testing methodology where the full AV stack (prediction + planning + control) is evaluated in simulation or real-world scenarios.

     See also: :doc:`Challenges in Evaluation </lecture4/lecture4>`.
