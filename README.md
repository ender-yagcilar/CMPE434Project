# CMPE434Project

## Introduction

In modern robotics, autonomous navigation in dynamic environments is a fundamental challenge. Robots not only need to find a path to their target but also adapt their motion to avoid collisions with moving obstacles. This term project addresses that challenge within a simulated environment using **MuJoCo** — a high-fidelity physics engine for robot simulation.

The primary goal of the project was to enable a robot to reach a predefined goal position without colliding with any obstacles in its environment. Unlike static settings, the simulation contains **dynamic obstacles**, making the problem more realistic and complex.

To solve this, a **hybrid motion planning approach** was implemented:
- **Probabilistic Roadmap (PRM)** was used as a **global planner** to generate a collision-free roadmap of the environment.
- **Artificial Potential Field (APF)** was employed as a **local planner** to reactively avoid dynamic obstacles.

This blog post walks through the motivation, methodology, implementation details, and results of the project.

## Why PRM and APF?

Choosing the right motion planning algorithms is critical for balancing performance, efficiency, and adaptability — especially in environments with dynamic obstacles.

### Why Probabilistic Roadmap (PRM) for Global Planning?

PRM is a sampling-based planner that works particularly well in **high-dimensional configuration spaces** and **static environments**. For this project, it was ideal for global planning because:
- It **preprocesses the static environment** to build a roadmap offline, which saves computation during execution.
- It is **scalable** and can efficiently handle complex spaces without requiring an explicit map decomposition.

### Why Artificial Potential Field (APF) for Local Planning?

APF was chosen for local planning due to its **real-time responsiveness** and **simplicity**:
- APF provides **fast reactions** to dynamic obstacles by treating them as repulsive forces, while the goal attracts the robot.
- It works well as a **local adjustment layer**, modifying the global path from PRM to avoid new or moving obstacles.
- It’s lightweight and easy to integrate with existing planners like PRM, making it a practical choice for combining global foresight with local agility.


By combining PRM and APF, the system benefits from both **efficient global path generation** and **adaptive local obstacle avoidance**, making it well-suited for navigating a dynamic MuJoCo environment.

