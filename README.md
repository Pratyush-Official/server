# 2026_servers
This repository contains the server-side implementation of the PID control system used in a larger autonomous robotics stack.

These nodes are designed to run as action servers, where the clients are action nodes inside a Behavior Tree (BT). The Behavior Tree sends motion goals, and these PID servers execute them using sensor or odometry feedback.

🧠 Architecture Overview
Behavior Tree (Action Nodes)
            ↓
      ROS 2 Action Client
            ↓
   ┌──────────────────────┐
   │   PID Action Server  │
   │  (This Repository)   │
   └──────────────────────┘
            ↓
        /cmd_vel
            ↓
      Low-Level Control

The Behavior Tree acts as the high-level decision maker.

Its action nodes send goals to:

/movement_sick

/movement_odom

These servers execute closed-loop PID control.

Velocity commands are published to /cmd_vel.

Included Servers
1️⃣ SICK-Based PID Server

Action: /movement_sick

Feedback source: /sick_data (Float32MultiArray)

Controls X and Y motion using distance sensor values

100 Hz PID loop

Stops when error threshold is reached

Uses MultiThreadedExecutor

2️⃣ Odometry-Based PID Server

Action: /movement_odom

Feedback source: /odom_data (Int32MultiArray)

Converts encoder counts to distance

Controls motion using odometry error

100 Hz PID loop

What This Repository Is

✔ Server-side motion execution layer
✔ Closed-loop PID control
✔ Action-based motion interface
✔ Designed to plug into Behavior Tree clients

What This Repository Is NOT

✘ It is not the Behavior Tree
✘ It does not generate goals
✘ It does not perform planning

It only executes goals sent by the BT action nodes.

How It Fits in a Larger Stack

Typical usage:

High-level planning → Behavior Tree

Behavior Tree action node → sends goal

This server → executes motion via PID

STM32 / motor driver → receives /cmd_vel

Python Plot Tool

Also included:

A simple Tkinter + Matplotlib GUI

Subscribes to Float32MultiArray

Plots index 0 in real-time

Useful for debugging sensor feedback

Build & Run
colcon build
source install/setup.bash

Run SICK server:

ros2 run <package_name> sick_pid

Run Odom server:

ros2 run <package_name> odom_pid
