---
sidebar_position: 2
title: Navigation 2 (Nav2)
---

# Navigation 2 (Nav2)

Navigation 2 is the state-of-the-art navigation stack for ROS 2, enabling autonomous robot navigation in complex environments.

## Overview

Nav2 provides complete navigation capabilities including:

- Global and local path planning
- Localization (AMCL)
- Costmap management
- Behavior trees for complex behaviors

## Core Components

- **Navigator**: Main executive that coordinates navigation
- **Planners**: Global and local path planning algorithms
- **Controllers**: Local trajectory generation and execution
- **Recovery**: Behaviors to recover from navigation failures

## Behavior Trees

Nav2 uses behavior trees for:

- Flexible navigation task composition
- Conditional execution of navigation behaviors
- Recovery from navigation failures
- Custom navigation logic implementation

## Configuration

Nav2 is configured through YAML files that define:

- Planner parameters
- Controller settings
- Costmap configurations
- Robot footprint and limits

## Best Practices

- Properly tune costmap parameters for your robot
- Test navigation in simulation before deployment
- Configure appropriate recovery behaviors