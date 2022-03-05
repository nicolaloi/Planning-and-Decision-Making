# Planning and Decision Making - Final 21

This exercise is the final graded exercise issued for the Fall semester of 2021.
It is not mandatory, but it can contribute up to 0.25 of your final grade only if it helps in improving it.

You can team up with other students and solve the graded exercise in groups of up to 4 people. 
Each member of the group will receive the same grade.

## Problem description
In 2020 a new spacecraft named _"PDM4AR"_ has been launched into deep space with the mission of bringing supplies to the interplanetary space station.
Unexpectedly a field of asteroids is surrounding the interplanetary station at the moment of arrival. 
You are in charge of writing a new planner for the spacecraft to safely complete the last stretch of the mission.
Your task is to implement an agent able to reach the docking area in the best possible way.

To test your agent, you are provided with a simulator able to perform closed loop simulations.
The simulator at each time step provides observations to the agent, and it expects commands in return. 

Note that the current simulator provides full state observability. 
That is, at each simulation step you will receive observations about your state and (if any) other moving obstacles which correspond to the real state of the simulation.
You assume no uncertainty about the observations.

![sim2agent](https://user-images.githubusercontent.com/18750753/144580159-d4d29506-03b2-49b9-b4b8-3cde701cc7d4.png)

More specifically, at each step the simulator will call the agent's `get_commands` method passing the latest observations as input and expecting commands in return.
The agent interface looks like this (see it in the `final21/agent.py` file):
```python
class Pdm4arAgent(Agent):
    """This is the PDM4AR agent.
    Do NOT modify the naming of the existing methods and the input/output types.
    Feel free to add additional methods, objects and functions that help you to solve the task"""

    def __init__(self,
                 goal: PolygonGoal,
                 static_obstacles: Sequence[StaticObstacle],
                 sg: SpacecraftGeometry):
        self.goal = goal
        self.static_obstacles = static_obstacles
        self.sg = sg
        self.name = None
        # todo implement here

    def on_episode_init(self, my_name: PlayerName):
        self.name = my_name

    def get_commands(self, sim_obs: SimObservations) -> SpacecraftCommands:
        """ This method is called by the simulator at each time step.

        This is how you can get your current state from the observations:
        my_current_state: SpacecraftState = sim_obs.players[self.name].state

        :param sim_obs:
        :return:
        """

        # todo implement here

        return SpacecraftCommands(acc_left=1, acc_right=1)
```
You can "navigate" the code (e.g., `Ctrl+left click` in Pycharm) to see the details of the structures of the observations, commands, and so on... 

The simulation terminates upon one of the following cases:
- The agent reaches the goal (you manage to bring the Spacecraft CoG inside the goal area)
- The agent crashes into an obstacle
- The maximum simulation time is reached

### Spacecraft model
The spacecraft has a left and right thruster at the back that can be activated to push the spacecraft forward or backward.
Note that applying differential thrust will also cause the spacecraft to rotate.

An illustrative figure  of the spacecraft model is shown below.

![spacecraft](https://user-images.githubusercontent.com/18750753/144763494-d0dc0d49-482c-4490-bca9-40149be51800.png)
The state of the spacecraft is described by:
```python
@dataclass
class SpacecraftState:
    x: float
    """ CoG x location [m] """
    y: float
    """ CoG y location [m] """
    psi: float
    """ Heading (yaw) [rad] """
    vx: float
    """ CoG longitudinal velocity [m/s] """
    vy: float
    """ CoG longitudinal velocity [m/s] """
    dpsi: float
    """ Heading (yaw) rate [rad/s] """
```
The commands (actuation) are:
```python
@dataclass
class SpacecraftCommands:
    acc_left: float
    """ linear acceleration of the left thruster [m/s^2] """
    acc_right: float
    """ linear acceleration of the right thruster [m/s^2]"""
```

Geometry and parameters of the spacecraft are accessible via the `SpacecraftGeometry` and the `SpacecraftParameters` class. 

Note that the simulator will enforce the following constraints:
- **Actuation limits**: The acceleration of each thruster is saturated in the interval [-10, 10] m/s^2
- **State constraints**: Linear velocities cannot exceed the interval [-50, 50] km/h
- **State constraints**: Angular velocities cannot exceed the interval [-2pi, 2pi] rad/s

If state constraints are violated, the simulator will set the commands to zero (unless they help to return within the physical constraints).

#### Suggestion for development
Take advantage of **pre-existing library**:
- Obstacle shapes are implemented using [Shapely](https://shapely.readthedocs.io/en/stable/), 
which already implements data structures and operations that you can take advantage of.
- Other scientific libraries are for example [numpy](https://numpy.org/), [scipy](https://www.scipy.org/),...


**Early development**: 
Note that running your agent paired in closed loop might be not the best way to build early prototypes.
Adding custom visualisation might be tedious and creating the animation might take a few seconds every time.
We suggest developing and test first your agent on a specific snapshot of the environment.
This, for example, allows you to add custom visualisations to debug your algorithm.
A simple example is provided in `test_agent.py`.

**Test on different instances**:
To avoid hard-coded solutions we will test your submissions on different instances of the environment.
You can make sure that your solution can deal with different instances of the world by changing the parameters that create the space/obstacles/goal region and different initial conditions in the file `exercises_def/final21/scenario.py`.

### Performance criteria
Your solution will be benchmarked against two scenarios. 
One containing only static obstacles (asteroids), one containing also dynamic obstacles (asteroids).
![image](https://user-images.githubusercontent.com/18750753/144765049-ffed6186-8269-4380-b382-a8e049ca7d39.png)
Once you run a simulation a report containing the visualisation of the episode and a few performance metrics is generated.
We will generate a ranking based on the performance criteria.

### Submission 
Code all your algorithms in files placed inside the `exercises/final21` folder. 
You are allowed to use any python library that can be installed simply via pip. 
You can add it as a dependency in the `requirements.txt` file.

#### What to submit
Only one member of your team should submit the final version of the code.
You must submit a zip file containing the following files:

- The folder `exercises/final21` with the relative python files containing your implementation
- The `requirements.txt` file.
- A `group.csv` file containing the list of your team members including (in order comma separated values): `legi number, email, name, surname.`

**Test** your submission locally first **with docker** because that is how they are going to be evaluated.

#### Where to submit
On Moodle, we will open an apposite tool.

#### When to submit
We will open the submission tool by the end of next week.
The ultimate deadline for submission is the 31st of December 2021 at 23:59 (CET).


