from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import TypeVar, Callable, Sequence, Generic

from reprep import Report


class ExIn(ABC):
    @abstractmethod
    def str_id(self) -> str:
        """Implement this to return the string for the report identifier"""
        pass


ExInT = TypeVar("ExInT", bound=ExIn)
"""Type of the exercise input, ideally implements the str_id"""
ExOutT = TypeVar("ExOutT")
"""Type of the exercise output"""


@dataclass
class Exercise(Generic[ExInT, ExOutT]):
    desc: str
    algorithm: Callable[[ExInT], ExOutT]
    report: Callable[[ExInT, ExOutT], Report]
    test_values: Sequence[ExInT] = field(default_factory=list)
