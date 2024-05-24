# ScientificPython

## Project Description

This project is developed as part of the curriculum at UPSSITECH, an engineering school in Toulouse, France. The project is carried out during a semester abroad at Pázmány Péter Catholic University (PPKE) in Budapest, Hungary. The goal of the project is to implement a system for mobile robot localization and control in a 2D environment using both odometry and landmark perception and in a Mujoco 3D simulation environment. Additionally, the system aims to integrate A* algorithm for path planning with obstacles.

```
Given a 2D plane environment equipped with three perfectly distinguishable landmarks previously located in an absolute manner,

1. in parallel to 2 below: sequentially locate a non-holonomic mobile robot in motion based on (1) its odometry; (2) the perception of the relative Cartesian coordinates of each of these landmarks (or the relative azimuth-distance pair to each landmark) from the robot; the self-perception and landmark perception aspect would be reduced to simulating movements and measurements relative to the landmarks (no computer vision techniques, image analysis, landmark detection-tagging, optical odometry, etc.); the localization engine would be the Kalman filter (non-linear extension);

2. in parallel to 1 above: command a non-holonomic mobile robot that is assumed to be precisely located in an absolute manner (2D position and absolute orientation) in such a way that it follows the trajectory (path + timing law) of a reference robot; mobile robot control techniques;

3. bring together the points 1 and 2 above in such a way that the tracking on a reference trajectory is performed on the localization obtained by Kalman instead of a presumed exact localization.
```

## Program Description

### Localization and Control
The program consists of a class called `CarController`, which is responsible for controlling a non-holonomic mobile robot in a 2D environment. The robot's localization is achieved through odometry and landmark perception using the Kalman filter. The program utilizes the Mujoco physics engine for simulation.

### Path Planning with A* Algorithm
In addition to the localization and control system, the program also includes an implementation of the A* algorithm for path planning. The `AStarPlanner` class generates a collision-free path from a start point to a goal point in a grid-based environment with obstacles.

## Future Improvements
One of the future improvements planned for this project is the integration of the A* algorithm with obstacle avoidance capabilities into the mobile robot control system. This enhancement will enable the robot to navigate dynamically changing environments while avoiding obstacles efficiently.

---
This README file was generated as part of the project undertaken at UPSSITECH, Toulouse, France, during a semester abroad at PPKE, Budapest, Hungary.
