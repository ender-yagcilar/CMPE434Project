# CMPE434Project

## Introduction

In modern robotics, autonomous navigation in dynamic environments is a fundamental challenge. Robots not only need to find a path to their target but also adapt their motion to avoid collisions with moving obstacles. This term project addresses that challenge within a simulated environment using **MuJoCo** — a high-fidelity physics engine for robot simulation.

The primary goal of the project was to enable a robot to reach a predefined goal position without colliding with any obstacles in its environment. Unlike static settings, the simulation contains **three dynamic obstacles** that move unpredictably, making the problem more realistic and complex.

To solve this, a **hybrid motion planning approach** was implemented:
- **Probabilistic Roadmap (PRM)** was used as a **global planner** to generate a collision-free roadmap of the environment.
- **Artificial Potential Field (APF)** was employed as a **local planner** to reactively avoid dynamic obstacles and fine-tune the path during execution.

This blog post walks through the motivation, methodology, implementation details, and results of the project.
