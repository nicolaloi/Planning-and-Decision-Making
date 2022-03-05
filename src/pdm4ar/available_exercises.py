from typing import Mapping

from frozendict import frozendict

from pdm4ar.exercises_def import *
from pdm4ar.exercises_def.structures import Exercise

available_exercises: Mapping[str, Callable[[], Exercise]] = frozendict(
    {
        "final21": get_final21,
    }
)
