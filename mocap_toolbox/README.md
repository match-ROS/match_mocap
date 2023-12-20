# Mocap Toolbox

## Table of Contents

- [About](#about)
- [Getting Started](#getting_started)
- [Usage](#usage)

## About <a name = "about"></a>

A Toolbox ROS-package for the qualisys motion capture system.

## Getting Started <a name = "getting_started"></a>

Copy this folder in the catkin_ws on the roscore-computer.

## Usage <a name = "usage"></a>

This section explains the different scripts in this package and how to use them.

### mocap_transformer.py
Transforms mocap poses to map frame (from mocap/* to mocap_map/*). Could also be used as a base class in an other python script.
Run via:

```bash
rosrun mocap_toolbox mocap_transformer.py 
```
