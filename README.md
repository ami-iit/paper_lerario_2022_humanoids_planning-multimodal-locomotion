<h1 align="center">
Whole-Body Trajectory Optimization for Robot Multimodal Locomotion
</h1>

<div align="center">

Giuseppe L'Erario, Gabriele Nava, Giulio Romualdi, Fabio Bergonti, Valentino Razza, Stefano Dafarra, Daniele Pucci

</div>

<p align="center">

<https://user-images.githubusercontent.com/29798643/168559242-ac97fa3e-31b6-468b-9f2a-42986b69e33c.mp4>

## Abstract

The general problem of planning feasible trajectories for multimodal robots is still an open challenge. This paper presents a whole-body trajectory optimisation approach that addresses this challenge by combining methods and tools developed for aerial and legged robots. First, robot models that enable the presented whole-body trajectory optimisation framework are presented. The key model is the so-called robot centroidal momentum, the dynamics of which is directly related to the models of the robot actuation for aerial and terrestrial locomotion. Then, the paper presents how these models can be employed in an optimal control problem to generate either terrestrial or aerial locomotion trajectories with a unified approach. The optimisation problem considers robot kinematics, momentum, thrust forces and their bounds. The overall approach is validated using the multimodal robot iRonCub, a flying humanoid robot that expresses a degree of terrestrial and aerial locomotion. To solve the associated optimal trajectory generation problem, we employ ADAM, a custom-made open-source library that implements a collection of algorithms for calculating rigid-body dynamics using CasADi.

## Results

### Take-off

The robot starts at equilibrium and the jet thrust is set to zero.
We set the initial CoM position at 0.57 m and the final one at 0.7 m.
The net-force transitions smoothly from contact forces to jet thrusts. The horizon is 70 knots long.

**Weights**

|Cost Term| $\ddot{r}$ | $\dot{r}$ | $\dot{h_{\omega}}$ | $h_{\omega}$   | $\dot{s}$ | $s - \bar{s}$ | $\dot{f}$ | $f$   | $\dot{T}$ | $T$   | $U$   |
|---------| -------- | ------- | ------- | --- | ------- | ----------- | ------- | --- | ------- | --- | ---- |
|Weight   | 1e5      | 1e7     | 1e4     | 1e4 | 1e5     | 1e6         | 1e0     | 5e1 | 0       | 0   | 1e-1 |

![03-15-09-54](https://user-images.githubusercontent.com/29798643/168566165-86c89196-38df-4dc1-852c-5a66bbc32901.gif)

### Take-off and landing

We test our approach is a vertical take-off and landing that shall be generated as a single instance of the optimisation problem. For this reason, we need to increase the number of knots of the optimisation horizon to 200. The initial and final CoM position is set to 0.57 m while at the intermediate knot we request a CoM height higher than 0.7 m. The robot decreases the contact forces and increases the thrust. The trajectory of the CoM reaches the maximum height and smoothly comes back to the initial position.

**Weights**

|Cost Term| $\ddot{r}$ | $\dot{r}$ | $\dot{h_{\omega}}$ | $h_{\omega}$   | $\dot{s}$ | $s - \bar{s}$ | $\dot{f}$ | $f$   | $\dot{T}$ | $T$   | $U$   |
|---------| -------- | ------- | ------- | --- | ------- | ----------- | ------- | --- | ------- | --- | ---- |
|Weight| 1e5      | 1e7     | 1e4     | 1e4 | 1e5     | 1e6         | 1e0     | 5e1 | 0       | 0   | 1e-1 |

![05-16-12-25](https://user-images.githubusercontent.com/29798643/168592535-43ec03f3-d1bc-4e21-8198-a42d5a6fcb56.gif)

### Walking to flight transition

We test our approach in a context in which two different locomotion patterns are mixed. The robot starts at an initial CoM position at 0.57 m and transitions from walking to flying locomotion going 0.9 m forward and reaching 0.7 m. The number of knots of the optimisation horizon is equal to 100. . The robot moves a few steps forward while increasing the thrust. Eventually, the contacts break and the robot detaches from the ground. Note that the contact sequence in the legged locomotion phase is not predefined and emerges directly from the formulation.

**Weights**

|Cost Term| $\ddot{r}$ | $\dot{r}$ | $\dot{h_{\omega}}$ | $h_{\omega}$   | $\dot{s}$ | $s - \bar{s}$ | $\dot{f}$ | $f$   | $\dot{T}$ | $T$   | $U$   |
|---------| -------- | ------- | ------- | --- | ------- | ----------- | ------- | --- | ------- | --- | --- |
|Weight| 1e5      | 1e7     | 1e4     | 1e4 | 1e5     | 1e7         | 0       | 1e0 | 0       | 2e3 | 0   |

![04-28-18-32](https://user-images.githubusercontent.com/29798643/168568487-d371eb08-1fd9-4af3-b44b-74f56b4658e3.gif)

### Jumping

To validate the approach for generating multimodal locomotion, the fourth scenario consists of the robot taking a jump. The number of optimisation knots is set to 60. The initial and the final CoM position height equal 0.57 m. We add an additional constraint on the CoM height at the middle-knot x[N/2] > 0.75m. The CoM height decreases and then accelerates vertically until the contacts break. After the flight phase, the robot lands and returns to the initial configuration.

**Weights**

|Cost Term| $\ddot{r}$ | $\dot{r}$ | $\dot{h_{\omega}}$ | $h_{\omega}$   | $\dot{s}$ | $s - \bar{s}$ | $\dot{f}$ | $f$   | $\dot{T}$ | $T$   | $U$   |
|---------| -------- | ------- | ------- | --- | ------- | ----------- | ------- | --- | ------- | --- | --- |
|Weight| 1e0      | 1e0     | 1e0     | 1e0 | 1e4     | 2e1         | 1e-1    | 1e0 | 0       | 0   | 0   |

![05-16-11-07](https://user-images.githubusercontent.com/29798643/168568591-9582e8ca-1399-44ac-8e1a-e31425b4af63.gif)

## Code

⚠️ The code is work in progress!

The skeleton of the varius cases is in the [runs folder](https://github.com/ami-iit/paper_lerario_2022_ral_planning-multimodal-locomotion/tree/main/runs)
