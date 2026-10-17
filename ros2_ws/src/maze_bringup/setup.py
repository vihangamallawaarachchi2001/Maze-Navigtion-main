import os
from glob import glob

from setuptools import setup

PKG = "maze_bringup"


def tree(src):
    """Install every file under src/ preserving sub-directories."""
    out = []
    for root, _, files in os.walk(src):
        if files:
            out.append((os.path.join("share", PKG, root), [os.path.join(root, f) for f in files]))
    return out


setup(
    name=PKG,
    version="1.0.0",
    packages=[PKG],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PKG]),
        ("share/" + PKG, ["package.xml"]),
        ("share/" + PKG + "/launch", glob("launch/*.py")),
        ("share/" + PKG + "/config", glob("config/*.yaml")),
        ("share/" + PKG + "/worlds", glob("worlds/*")),
        ("share/" + PKG + "/urdf", glob("urdf/*")),
        ("share/" + PKG + "/rviz", glob("rviz/*")),
    ] + tree("models") + tree("maps"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="IE4060 group",
    description="Maze navigation bring-up",
    license="MIT",
    entry_points={"console_scripts": [
        "goal_adapter = maze_bringup.goal_adapter:main",
        "run_trials = maze_bringup.run_trials:main",
        "motion_test = maze_bringup.motion_test:main",
        "mapping_drive = maze_bringup.mapping_drive:main",
    ]},
)
