# PDM4AR-exercises

RRT* path planning and MPC path tracking pipeline for the navigation of a spacecraft avoiding static and dynamic asteroids.

#

Project rated with full marks.

#

<img align="right" height="290" src="https://user-images.githubusercontent.com/79461707/156426758-994f43ea-c6e9-4a83-b154-8af7f89e2051.png"></img>


## Task

Control a spacecraft to safely reach the goal region, in an environment full of static and dynamic obstacles. 
The spacecraft has a left and right thruster at the back that can be activated to push the spacecraft forward or backward.
The simulator provides observations about the state of the spacecraft and of the moving obstacles which correspond to the real state of the simulation.
Optimize the total travelled distance, the final time, and the actuation effort.

<br/>

Further info: [Assignment.md](Assignment.md) or [this web page](https://idsc-frazzoli.github.io/PDM4AR-exercises/10-final21).

<br/>

## Pipeline

* To complete the task, an Informed RRT* is first applied to the static environment only, in order to have the global path the spacecraft has to follow at the beginning of the simulation. The algorithm is given a safety margin from the obstacles, and the number of iterations is proportional to the area of the environment. 

* A MPC is exploited to calculate the local trajectory to follow the global path obtained from the InformedRRT* algorithm. The global path is locally smoothed. The inputs of the MPC with *n*-step horizon are the pose and velocities of the spacecraft, the next *n* points of the smoothed path, and the *n* future positions of the dynamic obstacles (estimated through their current pose/velocity and dynamics). The spacecraft is constrained to remain inside the environment boundaries, and to not exceed the speed of 50 km/h.

An example of how the model works is shown in the following:


| *Example 1*  | *Example 2*  |  *Example 3* |
| :----------: | :----------: | :----------: |
| <img src="https://user-images.githubusercontent.com/79461707/156462358-10f9f32b-ded7-4b6a-9c37-c18220617494.png"/> | <img src="https://user-images.githubusercontent.com/79461707/156462386-3d27f2f3-669e-414f-9134-fbc28b89ed49.png"/> | <img src="https://user-images.githubusercontent.com/79461707/156427383-002e8268-8802-4dc0-98d9-16ca6323cd23.png"/>
| <video src="https://user-images.githubusercontent.com/79461707/156427435-a8b6ac57-cb6b-474a-ac07-c4134638f3d9.mp4" /> | <video src="https://user-images.githubusercontent.com/79461707/156427467-4a9a15d0-3d63-4ad9-b38d-01ba06c5f960.mp4" /> | <video src="https://user-images.githubusercontent.com/79461707/156427479-312f6e81-f16a-478d-add3-de01ce2eece4.mp4" /> |

## Setup

* The general simulation settings, like the size of the environment, the number, size and position of the static obstacles, the initial pose/velocity of the spacecraft, and the position of the goal region, can be modified in [src/pdm4ar/exercises_def/final21/scenario.py](src/pdm4ar/exercises_def/final21/scenario.py).  
 The number, size and initial pose/velocity of the dynamic obstacles can be modified in [src/pdm4ar/exercises_def/final21/sim_context.py](src/pdm4ar/exercises_def/final21/sim_context.py).

* The source code of the pipeline is inside the folder [src/pdm4ar/exercises/final21](src/pdm4ar/exercises/final21).


Run the pipeline and the simulation from the root directory with 

```shell
make run-final21
```

The pic of the Informed RRT* path is saved in [src/pdm4ar](src/pdm4ar). The video of the whole simulation is saved in the generated folder *out-docker/final21* as a *html* page.


The pipeline and the simulation run within a Docker container in Ubuntu 20.04.

