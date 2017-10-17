#! /usr/bin/env python
import unittest
import numpy as np
import criutils as cu
import tf.transformations as tr
# Messages
from geometry_msgs.msg import (Point, Quaternion, Pose, Vector3, Transform,
                                                                        Wrench)
from sensor_msgs.msg import RegionOfInterest


class Test_conversions(unittest.TestCase):
  def test_from_point(self):
    point_msg = Point(*np.random.sample(3))
    point_np = cu.conversions.from_point(point_msg)
    self.assertEqual(type(point_np), np.ndarray)
    np.testing.assert_almost_equal(point_np[0], point_msg.x)
    np.testing.assert_almost_equal(point_np[1], point_msg.y)
    np.testing.assert_almost_equal(point_np[2], point_msg.z)

  def test_from_pose(self):
    position = np.random.sample(3)
    orientation = tr.random_quaternion()
    pose_msg = Pose()
    pose_msg.position = Point(*position)
    pose_msg.orientation = Quaternion(*orientation)
    T = cu.conversions.from_pose(pose_msg)
    self.assertEqual(type(T), np.ndarray)
    self.assertEqual(T.shape, (4,4))
    # Check position and orientation
    np.testing.assert_allclose(T[:3,3], position)
    np.testing.assert_allclose(T[:3,:3],
                                      tr.quaternion_matrix(orientation)[:3,:3])

  def test_from_quaternion(self):
    orientation = tr.random_quaternion()
    quat_msg = Quaternion(*orientation)
    quat_np = cu.conversions.from_quaternion(quat_msg)
    self.assertEqual(type(quat_np), np.ndarray)
    np.testing.assert_allclose(quat_np, orientation)

  def test_from_roi(self):
    roi_msg = RegionOfInterest()
    roi_msg.x_offset = np.random.randint(1280/2)
    roi_msg.y_offset = np.random.randint(1024/2)
    roi_msg.width = np.random.randint(1280/2)
    roi_msg.height = np.random.randint(1024/2)
    roi_np = cu.conversions.from_roi(roi_msg)
    self.assertEqual(type(roi_np), list)
    self.assertEqual(len(roi_np), 2)

  def test_from_transform(self):
    translation = np.random.sample(3)
    rotation = tr.random_quaternion()
    tf_msg = Pose()
    tf_msg.position = Vector3(*translation)
    tf_msg.orientation = Quaternion(*rotation)
    T = cu.conversions.from_pose(tf_msg)
    self.assertEqual(type(T), np.ndarray)
    self.assertEqual(T.shape, (4,4))
    # Check position and orientation
    np.testing.assert_allclose(T[:3,3], translation)
    np.testing.assert_allclose(T[:3,:3], tr.quaternion_matrix(rotation)[:3,:3])

  def test_from_vector3(self):
    vector = np.random.sample(3)
    vect_msg = Vector3(*vector)
    vect_np = cu.conversions.from_vector3(vect_msg)
    self.assertEqual(type(vect_np), np.ndarray)
    np.testing.assert_allclose(vect_np, vector)

  def test_from_wrench(self):
    force = np.random.sample(3)
    torque = np.random.sample(3)
    wrench_msg = Wrench()
    wrench_msg.force = Vector3(*force)
    wrench_msg.torque = Vector3(*torque)
    wrench_np = cu.conversions.from_wrench(wrench_msg)
    self.assertEqual(type(wrench_np), np.ndarray)
    np.testing.assert_allclose(wrench_np[:3], force)
    np.testing.assert_allclose(wrench_np[3:], torque)

  def test_to_quaternion(self):
    quat_np = tr.random_quaternion()
    quat_msg = cu.conversions.to_quaternion(quat_np)
    self.assertEqual(type(quat_msg), Quaternion)
    np.testing.assert_almost_equal(quat_msg.x, quat_np[0])
    np.testing.assert_almost_equal(quat_msg.y, quat_np[1])
    np.testing.assert_almost_equal(quat_msg.z, quat_np[2])
    np.testing.assert_almost_equal(quat_msg.w, quat_np[3])

  def test_to_point(self):
    point_np = np.random.sample(3)
    point_msg = cu.conversions.to_point(point_np)
    self.assertEqual(type(point_msg), Point)
    np.testing.assert_almost_equal(point_msg.x, point_np[0])
    np.testing.assert_almost_equal(point_msg.y, point_np[1])
    np.testing.assert_almost_equal(point_msg.z, point_np[2])

  def test_to_pose(self):
    orientation = tr.random_quaternion()
    position = np.random.sample(3)
    T = tr.quaternion_matrix(orientation)
    T[:3,3] = position
    pose_msg = cu.conversions.to_pose(T)
    self.assertEqual(type(pose_msg), Pose)
    # Check position
    np.testing.assert_almost_equal(pose_msg.position.x, position[0])
    np.testing.assert_almost_equal(pose_msg.position.y, position[1])
    np.testing.assert_almost_equal(pose_msg.position.z, position[2])
    # Check orientation
    R = tr.quaternion_matrix([pose_msg.orientation.x, pose_msg.orientation.y,
                        pose_msg.orientation.z, pose_msg.orientation.w])[:3,:3]
    np.testing.assert_allclose(R, T[:3,:3])

  def test_to_roi(self):
    top_left = 200*np.random.sample(2)
    bottom_right = 800*np.random.sample(2) + 220
    roi_msg = cu.conversions.to_roi(top_left, bottom_right)
    self.assertEqual(type(roi_msg), RegionOfInterest)

  def test_to_transform(self):
    rotation = tr.random_quaternion()
    translation = np.random.sample(3)
    T = tr.quaternion_matrix(rotation)
    T[:3,3] = translation
    tf_msg = cu.conversions.to_transform(T)
    self.assertEqual(type(tf_msg), Transform)
    # Check translation
    np.testing.assert_almost_equal(tf_msg.translation.x, translation[0])
    np.testing.assert_almost_equal(tf_msg.translation.y, translation[1])
    np.testing.assert_almost_equal(tf_msg.translation.z, translation[2])
    # Check rotation
    R = tr.quaternion_matrix([tf_msg.rotation.x, tf_msg.rotation.y,
                        tf_msg.rotation.z, tf_msg.rotation.w])[:3,:3]
    np.testing.assert_allclose(R, T[:3,:3])

  def test_to_vector3(self):
    vect_np = np.random.sample(3)
    vect_msg = cu.conversions.to_vector3(vect_np)
    self.assertEqual(type(vect_msg), Vector3)
    np.testing.assert_almost_equal(vect_msg.x, vect_np[0])
    np.testing.assert_almost_equal(vect_msg.y, vect_np[1])
    np.testing.assert_almost_equal(vect_msg.z, vect_np[2])

  def test_to_wrench(self):
    wrench_np = np.random.sample(6)
    wrench_msg = cu.conversions.to_wrench(wrench_np)
    self.assertEqual(type(wrench_msg), Wrench)
    np.testing.assert_almost_equal(wrench_msg.force.x, wrench_np[0])
    np.testing.assert_almost_equal(wrench_msg.force.y, wrench_np[1])
    np.testing.assert_almost_equal(wrench_msg.force.z, wrench_np[2])
    np.testing.assert_almost_equal(wrench_msg.torque.x, wrench_np[3])
    np.testing.assert_almost_equal(wrench_msg.torque.y, wrench_np[4])
    np.testing.assert_almost_equal(wrench_msg.torque.z, wrench_np[5])
