#!/usr/bin/env python3
"""
Render robot description for gazebo.launch.py.

This avoids fragile shell quoting/pipes inside launch `Command()` substitutions.
It runs xacro, then strips XML declaration/comments/newlines to reduce the chance
of gazebo_ros2_control CLI parameter parsing failures.
"""

from __future__ import annotations

import os
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory


def _strip_text(s: str) -> str:
    # Import inline to avoid duplicating logic; call the same transformations as strip_urdf.py.
    import re

    s = re.sub(r"<\?xml[^>]*\?>", "", s, flags=re.IGNORECASE)
    s = re.sub(r"<!--.*?-->", "", s, flags=re.DOTALL)
    s = re.sub(r"<mujoco>.*?</mujoco>", "", s, flags=re.DOTALL | re.IGNORECASE)
    s = "".join(s.splitlines())
    s = s.replace('"', "'")
    return s


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: render_robot_description.py <rname>", file=sys.stderr)
        return 2

    rname = sys.argv[1].strip()
    if not rname:
        print("Error: rname is empty", file=sys.stderr)
        return 2

    pkg = f"{rname}_description"
    share = get_package_share_directory(pkg)
    xacro_file = os.path.join(share, "xacro", "robot.xacro")

    try:
        urdf = subprocess.check_output(["xacro", xacro_file], text=True)
    except subprocess.CalledProcessError as e:
        print(f"Error: xacro failed for {xacro_file}: {e}", file=sys.stderr)
        return e.returncode or 1

    urdf = _strip_text(urdf)

    # Fix mesh paths for G1 23DoF URDF.
    # The upstream g1_23dof_rev_1_0.urdf uses relative paths: filename="meshes/xxx.STL"
    # Gazebo can't resolve these unless they are absolute/model/package URIs.
    # For this repo, meshes live in g1_description, and g1_29dof URDF already uses package://.
    if rname == "g1_23":
        urdf = urdf.replace(
            "filename='meshes/", "filename='package://g1_description/meshes/"
        )

    sys.stdout.write(urdf)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
