#! /usr/bin/env python
import unittest
import criutils.rviz as criviz


class TestrvizModule(unittest.TestCase):
  def test_create_interactive_6dof(self):
    imarker = criviz.create_interactive_6dof('name')

  def test_create_interactive_mesh(self):
    imarker = criviz.create_interactive_mesh('name', 'resource')

  def test_create_mesh_marker(self):
    marker = criviz.create_mesh_marker(1, 'name', 'resource')

  def test_create_points_marker(self):
    points = [[1,2,3] for _ in range(10)]
    marker = criviz.create_points_marker(1, points)

  def test_create_text_marker(self):
    marker = criviz.create_text_marker(1, 'text')

  def test_get_safe_stamp(self):
    stamp = criviz.get_safe_stamp()
