<div align="center">

![logo](./images/inesctec_banner.png)

## INESC TEC
# Contributing to REUSE

</div>

Thanks for your interest in contributing! This document covers how to set up a
development environment, coding conventions, and the pull-request process.

## Development setup

The stack targets **Ubuntu 24.04** and **ROS 2 Jazzy**. It is a set of
colcon/ament packages built inside a ROS 2 workspace.

```bash
# 1. Source ROS 2
source /opt/ros/jazzy/setup.bash

# 2. Clone into a workspace
mkdir -p ~/dev_ws/src && cd ~/dev_ws/src
git clone <repository-url> Teleop_Manipulator
cd ~/dev_ws

# 3. Install dependencies, build and test
rosdep install --from-paths src --ignore-src -r -y
colcon build
colcon test && colcon test-result --verbose
```

See the [README](README.md#dependencies) for the full dependency list,
including the per-driver and input-script dependencies.

## Making changes

1. **Open an issue first** for anything beyond a trivial fix, so the approach
   can be discussed before you invest time in an implementation.
2. **Create a branch** off `master` named descriptively, e.g.
   `fix/kuka-udp-timeout` or `feat/add-fanuc-backend`.
3. **Keep commits focused** — one logical change per commit, with a clear
   commit message describing *why*, not just *what*.
4. **Match existing style** — follow the conventions already used in the
   file/package you're editing (naming, includes, parameter layout).
5. **New robots are new driver packages** — add a `pluginlib` backend under
   `robot_drivers/` (see the README, "Adding a new backend"); the node and
   `teleop_manipulators_core` should stay untouched.
6. **Update the README** if you change installation steps, parameters, launch
   files, or the package layout.

## Submitting a pull request

- Rebase on the latest `master` before opening the PR.
- Describe **what** changed and **why** in the PR description; link the
  related issue if one exists.
- Note any testing you did. Unit tests never talk to a robot, so a clear
  description of how you validated the change (e.g. "ran `teleop_mock`, then
  `ur_sim`, engaged the clutch and moved with the mouse") is genuinely useful
  to reviewers. Say which robot and controller version you used for hardware
  changes.
- A maintainer will review and may ask for changes before merging.

## Reporting bugs / requesting features

Open an issue and include:
- Commit, ROS 2 distro, OS, robot backend and robot/controller version.
- Steps to reproduce (for bugs), or the use case (for features).
- Relevant logs.

## Code of Conduct

This project follows the [Code of Conduct](CODE_OF_CONDUCT.md). By
participating, you're expected to uphold it.

## License

By contributing, you agree that your contributions will be licensed under the
project's [BSD 3-Clause License](LICENSE).
