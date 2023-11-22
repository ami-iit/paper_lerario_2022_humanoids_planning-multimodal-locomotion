<h1 align="center">
Whole-Body Trajectory Optimization for Robot Multimodal Locomotion
</h1>

<div align="center">

Giuseppe L'Erario, Gabriele Nava, Giulio Romualdi, Fabio Bergonti, Valentino Razza, Stefano Dafarra, Daniele Pucci

</div>

<p align="center">

<div align="center">

[<img src="https://user-images.githubusercontent.com/29798643/223752895-15b26a7b-272a-4310-8848-3a61d645c55e.png" width="800">](https://youtu.be/eWprjrLKxkY)

</div>

<p align="center">

<div align="center">
  <a href="#code"><b>Installation</b></a> |
  <a href="https://ieeexplore.ieee.org/document/10000241"><b>Paper</b></a> |
  <a href="https://www.youtube.com/watch?v=eWprjrLKxkY&t=7s"><b>Video</b></a>
</div>

<!-- just a backup of the old video -->
<!-- <https://user-images.githubusercontent.com/29798643/168559242-ac97fa3e-31b6-468b-9f2a-42986b69e33c.mp4> -->


## Abstract

The general problem of planning feasible trajectories for multimodal robots is still an open challenge. This paper presents a whole-body trajectory optimisation approach that addresses this challenge by combining methods and tools developed for aerial and legged robots. First, robot models that enable the presented whole-body trajectory optimisation framework are presented. The key model is the so-called robot centroidal momentum, the dynamics of which is directly related to the models of the robot actuation for aerial and terrestrial locomotion. Then, the paper presents how these models can be employed in an optimal control problem to generate either terrestrial or aerial locomotion trajectories with a unified approach. The optimisation problem considers robot kinematics, momentum, thrust forces and their bounds. The overall approach is validated using the multimodal robot iRonCub, a flying humanoid robot that expresses a degree of terrestrial and aerial locomotion. To solve the associated optimal trajectory generation problem, we employ ADAM, a custom-made open-source library that implements a collection of algorithms for calculating rigid-body dynamics using CasADi.

## Results

### Take-off

The robot starts at equilibrium and the jet thrust is set to zero.
We set the initial CoM position at 0.57 m and the final one at 0.7 m.
The net-force transitions smoothly from contact forces to jet thrusts. The horizon is 70 knots long.

#### Weights

| Cost Term | $\ddot{r}$ | $\dot{r}$ | $\dot{h_{\omega}}$ | $h_{\omega}$ | $\dot{s}$ | $s - \bar{s}$ | $\dot{f}$ | $f$ | $\dot{T}$ | $T$ | $U$  |
| --------- | ---------- | --------- | ------------------ | ------------ | --------- | ------------- | --------- | --- | --------- | --- | ---- |
| Weight    | 1e5        | 1e7       | 1e4                | 1e4          | 1e5       | 1e6           | 1e0       | 5e1 | 0         | 0   | 1e-1 |

![03-15-09-54](https://user-images.githubusercontent.com/29798643/168566165-86c89196-38df-4dc1-852c-5a66bbc32901.gif)

#### Ipop output

```bash
EXIT: Solved To Acceptable Level.
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  | 829.92ms ( 88.19us) 830.65ms ( 88.26us)      9411
       nlp_g  |  93.37 s (  9.92ms)  93.37 s (  9.92ms)      9411
  nlp_grad_f  | 512.66ms (172.27us) 512.85ms (172.33us)      2976
   nlp_jac_g  | 196.84 s ( 66.14ms) 196.85 s ( 66.14ms)      2976
       total  | 689.38 s (689.38 s) 689.52 s (689.52 s)         1
```

### Take-off and landing

We test our approach as a vertical take-off and landing that shall be generated as a single instance of the optimisation problem. For this reason, we need to increase the number of knots of the optimisation horizon to 200. The initial and final CoM position is set to 0.57 m while at the intermediate knot, we request a CoM height higher than 0.7 m. The robot decreases the contact forces and increases the thrust. The trajectory of the CoM reaches the maximum height and smoothly comes back to the initial position.

#### Weights

| Cost Term | $\ddot{r}$ | $\dot{r}$ | $\dot{h_{\omega}}$ | $h_{\omega}$ | $\dot{s}$ | $s - \bar{s}$ | $\dot{f}$ | $f$ | $\dot{T}$ | $T$ | $U$  |
| --------- | ---------- | --------- | ------------------ | ------------ | --------- | ------------- | --------- | --- | --------- | --- | ---- |
| Weight    | 1e5        | 1e7       | 1e4                | 1e4          | 1e5       | 1e6           | 1e0       | 5e1 | 0         | 0   | 1e-1 |

<!-- ![05-16-12-25](https://user-images.githubusercontent.com/29798643/168592535-43ec03f3-d1bc-4e21-8198-a42d5a6fcb56.gif) -->

https://github.com/ami-iit/paper_lerario_2022_humanoids_planning-multimodal-locomotion/assets/29798643/d0d620b2-0143-4215-b7d8-0b8bfb54bb32



#### Ipopt output

```bash
EXIT: Solved To Acceptable Level.
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  |   1.55 s (245.59us)   1.55 s (245.71us)      6319
       nlp_g  | 179.99 s ( 28.48ms) 179.99 s ( 28.48ms)      6319
  nlp_grad_f  |   1.23 s (489.44us)   1.23 s (489.61us)      2521
   nlp_jac_g  | 479.74 s (190.30ms) 479.75 s (190.30ms)      2521
       total  |   1.49ks (  1.49ks)   1.49ks (  1.49ks)         1
```

### Walking to flight transition

We test our approach in a context in which two different locomotion patterns are mixed. The robot starts at an initial CoM position at 0.57 m and transitions from walking to flying locomotion going 0.9 m forward and reaching 0.7 m. The number of knots of the optimisation horizon is equal to 100. The robot moves a few steps forward while increasing the thrust. Eventually, the contacts break and the robot detaches from the ground. Note that the contact sequence in the legged locomotion phase is not predefined and emerges directly from the formulation.

#### Weights

| Cost Term | $\ddot{r}$ | $\dot{r}$ | $\dot{h_{\omega}}$ | $h_{\omega}$ | $\dot{s}$ | $s - \bar{s}$ | $\dot{f}$ | $f$ | $\dot{T}$ | $T$ | $U$ |
| --------- | ---------- | --------- | ------------------ | ------------ | --------- | ------------- | --------- | --- | --------- | --- | --- |
| Weight    | 1e5        | 1e7       | 1e4                | 1e4          | 1e5       | 1e7           | 0         | 1e0 | 0         | 2e3 | 0   |

![04-28-18-32](https://user-images.githubusercontent.com/29798643/168568487-d371eb08-1fd9-4af3-b44b-74f56b4658e3.gif)

#### Ipopt output

```bash
EXIT: Solved To Acceptable Level.
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  | 377.97ms (104.76us) 378.35ms (104.86us)      3608
       nlp_g  |  49.13 s ( 13.62ms)  49.13 s ( 13.62ms)      3608
  nlp_grad_f  | 513.64ms (208.54us) 513.91ms (208.65us)      2463
   nlp_jac_g  | 224.63 s ( 91.20ms) 224.65 s ( 91.21ms)      2463
       total  |   1.65ks (  1.65ks)   1.65ks (  1.65ks)         1
```

### Jumping

To validate the approach for generating multimodal locomotion, the fourth scenario consists of the robot taking a jump. The number of optimisation knots is set to 60. The initial and the final CoM position height equal 0.57 m. We add an additional constraint on the CoM height at the middle-knot $x[N/2] > 0.75m$. The CoM height decreases and then accelerates vertically until the contacts break. After the flight phase, the robot lands and returns to the initial configuration.

#### Weights

| Cost Term | $\ddot{r}$ | $\dot{r}$ | $\dot{h_{\omega}}$ | $h_{\omega}$ | $\dot{s}$ | $s - \bar{s}$ | $\dot{f}$ | $f$ |
| --------- | ---------- | --------- | ------------------ | ------------ | --------- | ------------- | --------- | --- |
| Weight    | 1e0        | 1e0       | 1e0                | 1e0          | 1e4       | 2e1           | 1e-1      | 1e0 |

![05-16-11-07](https://user-images.githubusercontent.com/29798643/168568591-9582e8ca-1399-44ac-8e1a-e31425b4af63.gif)

#### Ipopt output

```bash
EXIT: Solved To Acceptable Level.
      solver  :   t_proc      (avg)   t_wall      (avg)    n_eval
       nlp_f  | 276.99ms ( 78.74us) 277.31ms ( 78.83us)      3518
       nlp_g  |  29.52 s (  8.39ms)  29.52 s (  8.39ms)      3518
  nlp_grad_f  | 156.72ms (153.34us) 156.79ms (153.42us)      1022
   nlp_jac_g  |  58.66 s ( 57.40ms)  58.66 s ( 57.40ms)      1022
       total  | 187.89 s (187.89 s) 187.97 s (187.97 s)         1
```

## Code

The iRonCub model and its meshes are stored in the [iRonCub software](https://github.com/ami-iit/ironcub_software.git) repository.

You need to install [**`git LFS`**](https://git-lfs.com/) and enable it.
Then, clone the [iRonCub software](https://github.com/ami-iit/ironcub_software.git) repository:

```bash
git clone https://github.com/ami-iit/ironcub_software.git --branch v1.0
```

in your `~/.bashrc` add the following lines, in order to load the robot model and the meshes:

```bash
export YARP_ROBOT_NAME=iRonCub-Mk1_1_v1
export IRONCUB_SOFTWARE_SOURCE_DIR=<path-to-ironcub-software>
export YARP_DATA_DIRS=${YARP_DATA_DIRS}:${IRONCUB_SOFTWARE_SOURCE_DIR}/models/iRonCub-Mk1_1/iRonCub/
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${IRONCUB_SOFTWARE_SOURCE_DIR}/models/
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${IRONCUB_SOFTWARE_SOURCE_DIR}/models/iRonCub-Mk1_1/iRonCub/robots
```

Clone this repository:

```bash
git clone git@github.com:ami-iit/paper_lerario_2022_humanoids_planning-multimodal-locomotion.git
```

Create an environment, and install the dependencies and the package:

```bash
cd paper_lerario_2022_humanoids_planning-multimodal-locomotion
conda config --add channels conda-forge # if you don't have the conda-forge channel already
mamba env create -f environment.yml
pip install .
```

Run from the root directory of the repository

```bash
show_trajectory --help
```

It will show the commands needed to visualize the trajectories and plots, e.g.:

```bash
show_trajectory transition
```

![code](https://user-images.githubusercontent.com/29798643/220925007-999c925c-036e-45f3-8879-8c00508a5657.png)


It will show the visualizations of the trajectories and plots, e.g.:

|⚠️ Warning |
|--|
|Note that to replicate the results, you need to install the hsl solvers (here we use `ma97`) that can be downloaded but not redistributed. Please check [here](https://www.hsl.rl.ac.uk/ipopt/). If you want to speed up the simulation you may install `IPOPT 3.13.4` with `CoinBrew` +  `HSL solver` as explained [here](https://gist.github.com/GiulioRomualdi/22fddb949e7b09bb53ca2ff72cbf8cb6)!|

To run the optimizations, run the scripts in the [runs folder](https://github.com/ami-iit/paper_lerario_2022_ral_planning-multimodal-locomotion/tree/main/runs), from the root, e.g.:

```bash
python runs/problem_take_off.py
```

## Citing this work

If you find the work useful, please consider citing:

```bibtex
@INPROCEEDINGS{10000241,
  author={L'Erario, Giuseppe and Nava, Gabriele and Romualdi, Giulio and Bergonti, Fabio and Razza, Valentino and Dafarra, Stefano and Pucci, Daniele},
  booktitle={2022 IEEE-RAS 21st International Conference on Humanoid Robots (Humanoids)},
  title={Whole-Body Trajectory Optimization for Robot Multimodal Locomotion},
  year={2022},
  volume={},
  number={},
  pages={651-658},
  doi={10.1109/Humanoids53995.2022.10000241}}
```

## Maintainer

This repository is maintained by:

|                                                              |                                                      |
| :----------------------------------------------------------: | :--------------------------------------------------: |
| [<img src="https://github.com/Giulero.png" width="40">](https://github.com/Giulero) | [@Giulero](https://github.com/Giulero) |
