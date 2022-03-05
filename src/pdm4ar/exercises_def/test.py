from typing import NoReturn, Any

from reprep import Report

from pdm4ar.exercises_def import logger
from pdm4ar.exercises_def.structures import Exercise


def _test_algo(algo_in: str) -> NoReturn:
    logger.info("Test exercise ran correctly!")


def _test_rep(algo_in: str, alg_out: Any) -> Report:
    r = Report()
    r.text("Info", text=algo_in)
    return r


def get_test_exercise() -> Exercise:
    desc = "This is the test exercise.\nEverything looks good, time to go out in the sun."
    return Exercise[str, Any](
        desc=desc,
        algorithm=_test_algo,
        report=_test_rep,
        test_values=[
            "This is the test exercise.\nEverything looks good, time to go out in the sun.",
        ],
    )
