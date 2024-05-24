# ScientificPython

## Project Description

This project is developed as part of the curriculum at UPSSITECH, an engineering school in Toulouse, France. The project is carried out during a semester abroad at Pázmány Péter Catholic University (PPKE) in Budapest, Hungary. The goal of the project is to implement a system for mobile robot localization and control in a 2D environment using both odometry and landmark perception and in a Mujoco 3D simulation environment. Additionally, the system aims to integrate A* algorithm for path planning with obstacles.

## Program Description

### Localization and Control
The program consists of a class called `CarController`, which is responsible for controlling a non-holonomic mobile robot in a 2D environment. The robot's localization is achieved through odometry and landmark perception using the Kalman filter. The program utilizes the Mujoco physics engine for simulation.

### Path Planning with A* Algorithm
In addition to the localization and control system, the program also includes an implementation of the A* algorithm for path planning. The `AStarPlanner` class generates a collision-free path from a start point to a goal point in a grid-based environment with obstacles.

## Future Improvements
One of the future improvements planned for this project is the integration of the A* algorithm with obstacle avoidance capabilities into the mobile robot control system. This enhancement will enable the robot to navigate dynamically changing environments while avoiding obstacles efficiently.

---
This README file was generated as part of the project undertaken at UPSSITECH, Toulouse, France, during a semester abroad at PPKE, Budapest, Hungary.
