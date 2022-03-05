import os
import re

from decent_params import DecentParams
from quickapp import QuickAppContext, QuickApp
from zuper_commons.text import expand_string
from zuper_commons.types import ZValueError

from pdm4ar.available_exercises import available_exercises


class Pdm4arExercise(QuickApp):
    """Main Experiments runner"""

    def define_options(self, params: DecentParams):
        params.add_string("exercises", default="test")

    def define_jobs_context(self, context: QuickAppContext):

        do_exercises = self.get_options().exercises.split(";")
        do_exercises = expand_string(do_exercises, list(available_exercises))

        for exercise in do_exercises:
            if exercise not in available_exercises:
                raise ZValueError(f"Cannot find {exercise!r}", available=set(available_exercises))

        for exercise in do_exercises:
            ex = available_exercises[exercise]()
            c = context.child(exercise, extra_report_keys=dict(Exercise=exercise))
            for i, alg_in in enumerate(ex.test_values):
                try:
                    i_str = alg_in.str_id() + str(i)
                except:
                    i_str = str(i)
                c = c.child(i_str, extra_report_keys=dict(TestID=i_str))
                alg_out = c.comp(ex.algorithm, alg_in)
                report = c.comp(ex.report, alg_in, alg_out)
                c.add_report(report, report_type="PDM4AR")


def exercise_without_compmake(exercise: str):
    if exercise not in available_exercises:
        raise ZValueError(f"Cannot find {exercise!r}", available=set(available_exercises))
    repo_dir = __file__
    src_folder = "src"
    assert src_folder in repo_dir, repo_dir
    repo_dir = re.split(src_folder, repo_dir)[0]
    assert os.path.isdir(repo_dir)
    out = os.path.join(repo_dir, "out")

    ex = available_exercises[exercise]()
    for i, alg_in in enumerate(ex.test_values):
        try:
            i_str = alg_in.str_id() + str(i)
        except:
            i_str = str(i)
        alg_out = ex.algorithm(alg_in)
        report = ex.report(alg_in, alg_out)
        report_file = os.path.join(out, f"{exercise}-{i_str}.html")
        report.to_html(report_file)


exercise_main = Pdm4arExercise.get_sys_main()
