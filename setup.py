from setuptools import setup

install_requires = []

module = "pdm4ar"
package = "pdm4ar"
src = "src"

setup(
    name=package,
    package_dir={"": src},
    packages=[module],
    version="0.0.0",
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "pdm4ar-exercise = pdm4ar:exercise_main",
        ]
    },
    install_requires=install_requires,
)
