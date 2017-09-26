#! /usr/bin/env python
import rospy
import numpy as np
from . import conversions as criconv
# Messages
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import (InteractiveMarker,
                                              InteractiveMarkerControl, Marker)


def create_interactive_6dof(name, color=(1,0,0,1), frame_id='base_link',
                                                    transform=None, scale=0.05):
  if transform is None:
    transform = np.eye(4)
  int_marker = InteractiveMarker()
  int_marker.header.stamp = get_safe_stamp()
  int_marker.header.frame_id = frame_id
  int_marker.name = name
  int_marker.scale = scale
  int_marker.pose = criconv.to_pose(transform)
  # Move X
  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 1
  control.orientation.y = 0
  control.orientation.z = 0
  control.name = 'move_x'
  control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
  int_marker.controls.append(control)
  # Move Y
  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 0
  control.orientation.y = 0
  control.orientation.z = 1
  control.name = 'move_y'
  control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
  int_marker.controls.append(control)
  # Move Z
  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 0
  control.orientation.y = 1
  control.orientation.z = 0
  control.name = 'move_z'
  control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
  int_marker.controls.append(control)
  # Rotate X
  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 1
  control.orientation.y = 0
  control.orientation.z = 0
  control.name = 'rotate_x'
  control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
  int_marker.controls.append(control)
  # Rotate Y
  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 0
  control.orientation.y = 0
  control.orientation.z = 1
  control.name = 'rotate_y'
  control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
  int_marker.controls.append(control)
  # Rotate Z
  control = InteractiveMarkerControl()
  control.orientation.w = 1
  control.orientation.x = 0
  control.orientation.y = 1
  control.orientation.z = 0
  control.name = 'rotate_z'
  control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
  int_marker.controls.append(control)
  return int_marker

def create_interactive_mesh(name, resource, color=(1,0,0,1),
                                frame_id='base_link', transform=None, scale=1):
  if transform is None:
    transform = np.eye(4)
  # Start with the interactive control
  int_marker = create_interactive_6dof(name, color, frame_id, transform)
  # Mesh control
  control = InteractiveMarkerControl()
  control.always_visible = True
  control.interaction_mode = InteractiveMarkerControl.NONE
  # Mesh marker
  marker = Marker()
  marker.type = Marker.MESH_RESOURCE
  marker.scale = Vector3(*[scale for _ in range(3)])
  marker.color = ColorRGBA(*color)
  marker.action = Marker.ADD
  marker.mesh_resource = resource
  # Add mesh control
  control.markers.append(marker)
  int_marker.controls.append( control )
  return int_marker

def create_mesh_marker(mrkid, name, resource, color=(1,0,0,1), ns='',
                                frame_id='base_link', transform=None, scale=1):
  if transform is None:
    transform = np.eye(4)
  marker = Marker()
  marker.header.stamp = get_safe_stamp()
  marker.header.frame_id = frame_id
  marker.ns = ns
  marker.id = mrkid
  marker.type = Marker.MESH_RESOURCE
  marker.pose = criconv.to_pose(transform)
  marker.scale = Vector3(*[scale for _ in range(3)])
  marker.color = ColorRGBA(*color)
  marker.action = Marker.ADD
  marker.mesh_resource = resource
  return marker

def create_points_marker(mrkid, points, size=1e-3, color=(1,0,0,1), ns='',
                                          frame_id='base_link', transform=None):
  if transform is None:
    transform = np.eye(4)
  marker = Marker()
  marker.header.stamp = get_safe_stamp()
  marker.header.frame_id = frame_id
  marker.ns = ns
  marker.id = mrkid
  marker.type = Marker.POINTS
  marker.pose = criconv.to_pose(transform)
  marker.scale = Vector3(size, size, 0) # (width, height, unused)
  marker.color = ColorRGBA(*color)
  marker.action = Marker.ADD
  # Append the points
  for point in points:
    marker.points.append(criconv.to_point(point))
  return marker

def create_text_marker(mrkid, text, size=0.02, color=(1,0,0,1), ns='',
                                          frame_id='base_link', position=None):
  transform = np.eye(4)
  if position is not None:
    transform[:3,3] = position
  marker = Marker()
  marker.header.stamp = get_safe_stamp()
  marker.header.frame_id = frame_id
  marker.ns = ns
  marker.id = mrkid
  marker.type = Marker.TEXT_VIEW_FACING
  marker.text = text
  marker.pose = criconv.to_pose(transform)
  marker.scale.z = size
  marker.color = ColorRGBA(*color)
  marker.action = Marker.ADD
  return marker

def get_safe_stamp():
  try:
    stamp = rospy.Time.now()
  except rospy.ROSInitException:
    stamp = rospy.Time(0)
  return stamp
