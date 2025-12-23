#!/usr/bin/env python3
"""
Post-process URDF for gazebo_ros2_control.

Why:
  gazebo_ros2_control forwards robot_description to controller_manager via a CLI param override.
  ROS 2 rcl argument parsing can fail when the URDF contains XML declaration and/or XML comments.

What:
  - remove XML declaration
  - remove XML comments
  - remove Mujoco-specific <mujoco>...</mujoco> block (not needed for Gazebo)
  - remove newlines (emit a single line)
  - replace double quotes with single quotes (XML remains valid; YAML parsing becomes more robust)
"""

from __future__ import annotations

import re
import sys


def main() -> int:
    s = sys.stdin.read()
    # XML declaration
    s = re.sub(r"<\?xml[^>]*\?>", "", s, flags=re.IGNORECASE)
    # XML comments (multiline)
    s = re.sub(r"<!--.*?-->", "", s, flags=re.DOTALL)
    # Mujoco extension block (multiline)
    s = re.sub(r"<mujoco>.*?</mujoco>", "", s, flags=re.DOTALL | re.IGNORECASE)
    # Compact to single line
    s = "".join(s.splitlines())
    # Make YAML parsing more robust for '--param robot_description:=...'
    # (ROS 2 CLI param override uses YAML parser and is sensitive to embedded double quotes)
    s = s.replace('"', "'")
    sys.stdout.write(s)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
