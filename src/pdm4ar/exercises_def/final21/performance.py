from dataclasses import dataclass
from math import sqrt

import numpy as np
from dg_commons import iterate_with_dt, seq_integrate, DgSampledSequence, seq_differentiate
from dg_commons.sim import PlayerLog
from dg_commons.sim.models.spacecraft import SpacecraftState
from dg_commons.sim.simulator import SimContext

from pdm4ar.exercises_def.final21.sim_context import PDM4AR


@dataclass(frozen=True)
class EpisodeOutcome:
    goal_reached: bool
    """Whether the goal was reached or not."""
    has_collided: bool
    """Whether the robot has collided with an obstacle."""
    distance_travelled: float
    """The length of the path travelled by the robot"""
    episode_duration: float
    """The time it took till the end the simulation."""
    actuation_effort: float
    """Integral of the commands sent to the robot normalized by the time taken."""
    max_acc_lat: float
    """Maximum lateral acceleration of the robot."""
    max_acc_long: float
    """Maximum longitudinal acceleration of the robot."""
    avg_computation_time: float
    """Average computation time of the get_commands method."""

    def __repr__(self):
        repr: str = ""
        for k, v in self.__dict__.items():
            value = f"{v:>5.2f}" if isinstance(v, float) else f"{v:>5}"
            repr += f"\t{k:<20}=\t" + value + ",\n"

        return f"EpisodeOutcome(\n" + repr + "\n)"


def evaluate_episode(sim_context: SimContext) -> EpisodeOutcome:
    assert PDM4AR in sim_context.log, "Cannot find player name in sim_log"
    agent_log: PlayerLog = sim_context.log[PDM4AR]
    states: DgSampledSequence[SpacecraftState] = agent_log.states

    # if the last state of the sim is inside the goal
    last_state = states.values[-1]
    has_reached_the_goal: bool = sim_context.missions[PDM4AR].is_fulfilled(last_state)

    # collision
    has_collided = True if len(sim_context.collision_reports) > 0 else False

    # distance travelled
    dist: float = 0
    for it in iterate_with_dt(states):
        dist += sqrt((it.v1.x - it.v0.x) ** 2 + (it.v1.y - it.v0.y) ** 2)

    # time duration
    duration = float(states.get_end() - states.get_start())

    # actuation effort
    abs_acc = agent_log.actions.transform_values(lambda x: abs(x.acc_left) + abs(x.acc_right))
    actuation_effort = seq_integrate(abs_acc).values[-1] / duration

    # smoothness
    dsatate = seq_differentiate(states)
    dstate_acc_lat = dsatate.transform_values(lambda x: abs(x.vy))
    dstate_acc_long = dsatate.transform_values(lambda x: abs(x.vx))

    max_acc_lat = max(dstate_acc_lat.values)
    max_acc_long = max(dstate_acc_long.values)

    # computation time
    avg_comp_time = np.average(agent_log.info.values)

    return EpisodeOutcome(
        goal_reached=has_reached_the_goal,
        has_collided=has_collided,
        distance_travelled=dist,
        episode_duration=duration,
        actuation_effort=actuation_effort,
        max_acc_lat=max_acc_lat,
        max_acc_long=max_acc_long,
        avg_computation_time=avg_comp_time
    )
