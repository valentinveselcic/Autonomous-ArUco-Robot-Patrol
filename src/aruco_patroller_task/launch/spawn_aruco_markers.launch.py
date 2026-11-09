#!/usr/bin/env python3

import os
from math import pi  # if you want to use it somewhere
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_markers_from_yaml():
    pkg_share = get_package_share_directory("aruco_patroller_task")
    yaml_path = os.path.join(pkg_share, "config", "aruco_markers.yaml")

    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"Marker config YAML not found: {yaml_path}")

    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f) or {}

    markers = data.get("markers", [])
    if not markers:
        print(f"[spawn_aruco_markers] No 'markers' list in {yaml_path}")
    return markers, pkg_share


def generate_launch_description():
    markers, pkg_share = load_markers_from_yaml()

    spawn_nodes = []

    for m in markers:
        mid = m["id"]
        x = float(m.get("x", 0.0))
        y = float(m.get("y", 0.0))
        z = float(m.get("z", 0.0))
        roll = float(m.get("roll", 0.0))
        pitch = float(m.get("pitch", 0.0))
        yaw = float(m.get("yaw", 0.0))

        sdf_path = os.path.join(
            pkg_share, "models", f"aruco_marker_{mid}", "model.sdf"
        )

        if not os.path.exists(sdf_path):
            print(
                f"[spawn_aruco_markers] WARNING: SDF for marker {mid} not found at {sdf_path}"
            )
            continue

        spawn_nodes.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                name=f"spawn_aruco_{mid}",
                output="screen",
                arguments=[
                    "-file",
                    sdf_path,
                    "-name",
                    f"aruco_marker_{mid}",
                    "-allow_renaming",
                    "false",
                    "-x",
                    str(x),
                    "-y",
                    str(y),
                    "-z",
                    str(z),
                    "-R",
                    str(roll),
                    "-P",
                    str(pitch),
                    "-Y",
                    str(yaw),
                ],
            )
        )

    return LaunchDescription(spawn_nodes)