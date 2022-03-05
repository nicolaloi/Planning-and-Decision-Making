import contracts

from pdm4ar.app import exercise_without_compmake
import argparse

if __name__ == "__main__":
    contracts.disable_all()
    exercise_without_compmake("final21")
