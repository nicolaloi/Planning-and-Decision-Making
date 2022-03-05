from dataclasses import dataclass
from typing import Any

from dg_commons.sim.simulator import SimContext, Simulator
from dg_commons.sim.simulator_animation import create_animation
from reprep import Report, MIME_GIF

from pdm4ar.exercises_def import Exercise, ExIn
from pdm4ar.exercises_def.final21.performance import evaluate_episode
from pdm4ar.exercises_def.final21.sim_context import get_sim_context_static, get_sim_context_dynamic


@dataclass
class TestValueExFinal21(ExIn):
    sim_context: SimContext

    def str_id(self) -> str:
        return str(self.sim_context.description)


def generate_report(sim_context: SimContext) -> Report:
    r = Report("EpisodeVisualisation")
    gif_viz = r.figure(cols=1)
    with gif_viz.data_file("Animation", MIME_GIF) as fn:
        create_animation(file_path=fn,
                         sim_context=sim_context,
                         figsize=(16, 16),
                         dt=50, dpi=120,
                         plot_limits=None)

    episode_eval = evaluate_episode(sim_context)
    r.text("EpisodeEvaluation", str(episode_eval))

    return r


def ex_final21_report(ex_in: TestValueExFinal21, ex_out=None) -> Report:
    r = Report("Final21-" + ex_in.str_id())
    sim_context = ex_in.sim_context

    sim = Simulator()
    sim.run(sim_context)
    r.add_child(generate_report(sim_context=sim_context))
    return r


def algo_placeholder(ex_in):
    return None


def get_final21() -> Exercise:
    seed = 1
    test_values = [#TestValueExFinal21(get_sim_context_static(seed)),
                   TestValueExFinal21(get_sim_context_dynamic(seed)),
                   ]

    return Exercise[TestValueExFinal21, Any](
        desc="Graded exercise for Fall21 course",
        algorithm=algo_placeholder,
        report=ex_final21_report,
        test_values=test_values,
    )
