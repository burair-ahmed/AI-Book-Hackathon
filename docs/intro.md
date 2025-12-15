---
id: intro
title: Introduction
sidebar_position: 1
---

# Introduction: Physical AI & Humanoid Robotics

Welcome to **Physical AI & Humanoid Robotics: Embodied Intelligence in the Real World**.

This textbook bridges the gap between high-level AI software—like Large Language Models (LLMs) and Vision-Language Models (VLMs)—and the physical reality of embodied robotic systems.

## What You Will Build

By the end of this book, you will have built a complete **"Brain-to-Body"** stack for a humanoid robot. This includes:

1.  **A Nervous System**: A distributed ROS 2 graph for high-frequency control.
2.  **A Digital Twin**: A physics-accurate simulation in Gazebo and NVIDIA Isaac Sim.
3.  **A Cortex**: A VLA (Vision-Language-Action) model capable of understanding natural language commands.
4.  **Embodied Intelligence**: A system that can "See, Plan, and Act" in the real world.

## Prerequisites

This book assumes you are comfortable with:
*   **Python Programming**: Classes, asynchronous programming, and typing.
*   **Basic Machine Learning**: Concepts like inference, tensors, and models.
*   **Command Line Interface**: Navigating Linux terminals.

**We do NOT assume:**
*   Prior experience with ROS or Robotics.
*   Access to expensive hardware (we provide simulation alternatives).

## The Toolchain

We adhere to a strict **Linux-First** philosophy for maximum compatibility with industrial robotic standards.

| Component | Standard | Notes |
| :--- | :--- | :--- |
| **OS** | **Ubuntu 22.04 LTS** | The standard for ROS 2 Humble/Iron. |
| **Middleware** | **ROS 2 (Humble/Iron)** | The communication backbone. |
| **Simulation** | **Gazebo / Isaac Sim** | Testing grounds for digital twins. |
| **Hardware** | **NVIDIA Jetson / RTX** | GPU acceleration for AI workloads. |

:::warning Windows & MacOS Users
Robotics is a Linux-native domain. While you *can* use WSL2 (Windows) or Docker (Mac), we **strongly recommend** a native Ubuntu 22.04 dual-boot simulation machine for the best experience.
:::

## How to Read This Book

The content is organized into 5 Modules:

1.  **The Nervous System**: Mastering ROS 2.
2.  **Digital Twins**: Building physics simulations.
3.  **The Isaac Brain**: Leveraging GPU acceleration.
4.  **Vision-Language-Action**: Connecting LLMs to robots.
5.  **Capstone**: Integrating the full Humanoid stack.

Let's begin.
