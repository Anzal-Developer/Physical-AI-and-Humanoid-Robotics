---
sidebar_position: 3
title: URDF Basics
---

# URDF Basics

This module introduces Unified Robot Description Format (URDF), the standard for representing robot models in ROS.

## What is URDF?

URDF (Unified Robot Description Format) is an XML format that describes a robot's physical properties including:
- Kinematic and dynamic properties
- Visual and collision models
- Joint definitions and limits
- Inertial properties

## URDF Structure

A basic URDF file consists of:
- **Links**: Rigid parts of the robot (e.g., chassis, wheels)
- **Joints**: Connections between links (e.g., rotating, prismatic)
- **Visual**: How the robot appears in simulation
- **Collision**: How the robot interacts physically
- **Inertial**: Mass, center of mass, and inertia properties

## Creating a Simple Robot Model

A basic URDF robot model includes:

```xml
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
</robot>
```

## Best Practices

- Use meaningful names for links and joints
- Define proper inertial properties for accurate simulation
- Use appropriate collision geometries
- Organize complex robots with xacro macros