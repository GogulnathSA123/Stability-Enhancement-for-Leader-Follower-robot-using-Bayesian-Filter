# Stability-Enhancement-for-Leader-Follower-robot-using-Bayesian-Filter


In our leader–follower system, we used five infrared (IR) sensors on the follower robot to detect and track the leader’s position. Since IR sensor readings are noisy and can fluctuate due to surface reflections and environmental disturbances, directly using raw measurements caused instability and oscillations in the follower’s motion.
To address this, we implemented a Bayesian sensor fusion approach. The measurements from all five IR sensors were fused probabilistically to compute the posterior probability distribution of the leader’s relative position.

Using Bayes’ theorem:

Each IR sensor provided a likelihood of the leader being in a certain direction/region.

These likelihoods were combined with a prior belief (prediction from the previous time step).

The result was a posterior probability estimate representing the most probable location of the leader.

This posterior estimate was then used for control of the follower robot.
By using probabilistic fusion instead of raw sensor values, we achieved:

Reduced uncertainty in leader detection

Smoother heading corrections

Improved inter-robot distance regulation

Enhanced overall closed-loop stability

Thus, fusing data from the five IR sensors through Bayesian filtering significantly improved the stability and robustness of the leader–follower system.
