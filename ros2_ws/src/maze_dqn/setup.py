from glob import glob

from setuptools import setup

PKG = "maze_dqn"
setup(
    name=PKG,
    version="1.0.0",
    packages=[PKG],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PKG]),
        ("share/" + PKG, ["package.xml"]),
        ("share/" + PKG + "/config", glob("config/*.yaml")),
        ("share/" + PKG + "/checkpoints", glob("checkpoints/*.pt") + glob("checkpoints/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="IE4060 group",
    description="DQN route planner",
    license="MIT",
    entry_points={"console_scripts": [
        "train = maze_dqn.train:main",
        "evaluate = maze_dqn.evaluate:main",
    ]},
)
