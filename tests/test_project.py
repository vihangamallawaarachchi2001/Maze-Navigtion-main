"""Geometry and asset checks independent of a running ROS graph."""
import ast
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from maze_geometry import clearance, world_sdf


def test_world_matches_generator_and_has_collision_walls():
    world = ROOT / 'ros2_ws/src/maze_bringup/worlds/assignment_maze.sdf'
    assert world.read_text() == world_sdf()
    maze = ET.parse(world).find(".//model[@name='maze']")
    assert len(maze.findall('link')) == 18
    for link in maze.findall('link'):
        assert link.find('collision/geometry/box/size').text == link.find('visual/geometry/box/size').text
    assert world.with_suffix('.world').read_bytes() == world.read_bytes()


def test_spawn_and_goals_have_turning_clearance():
    cfg = ROOT / 'ros2_ws/src/maze_bringup/config'
    spawn = yaml.safe_load((cfg / 'safe_spawn.yaml').read_text())
    assert clearance(spawn['safe_x'], spawn['safe_y']) == pytest.approx(0.225)
    goals = yaml.safe_load((cfg / 'evaluation_goals.yaml').read_text())['scenarios']
    assert len(goals) == 3
    for goal in goals:
        assert clearance(*goal['goal_map'][:2]) > 0.13


def test_python_syntax_and_map_installation():
    for path in list((ROOT / 'ros2_ws/src').rglob('*.py')) + list((ROOT / 'tools').glob('*.py')):
        ast.parse(path.read_text(), filename=str(path))
    share_maps = ROOT / 'ros2_ws/src/maze_bringup/maps'
    for kind, name in [('slam', 'maze_slam'), ('reference', 'ground_truth_maze')]:
        path = share_maps / kind / (name + '.yaml')
        meta = yaml.safe_load(path.read_text())
        assert (path.parent / meta['image']).is_file()
