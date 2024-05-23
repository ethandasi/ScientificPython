import numpy as np
import matplotlib.pyplot as plt

class ReferenceTrajectory:
    def __init__(self, path):
        self.path = path
        self.num_waypoints = len(path)
        self.current_waypoint_index = 0

    def get_current_waypoint(self):
        return self.path[self.current_waypoint_index]

    def advance_waypoint(self):
        self.current_waypoint_index = min(self.current_waypoint_index + 1, self.num_waypoints - 1)

    def is_at_end(self):
        return self.current_waypoint_index == self.num_waypoints - 1

class MobileRobot:
    def __init__(self, initial_pose):
        self.pose = initial_pose
        self.v = 0.0  # Vitesse linéaire
        self.omega = 0.0  # Vitesse angulaire

    def update_pose(self, v, omega, dt):
        self.pose[0] += v * np.cos(self.pose[2]) * dt
        self.pose[1] += v * np.sin(self.pose[2]) * dt
        self.pose[2] += omega * dt

    def follow_trajectory(self, reference_trajectory, Kp, Ki, dt):
        integral_error = 0.0
        while not reference_trajectory.is_at_end():
            target_pose = reference_trajectory.get_current_waypoint()
            error = target_pose - self.pose[:2]  # Erreur de position
            integral_error += error * dt
            control_v = Kp * error[0] + Ki * integral_error[0]  # Commande de vitesse linéaire
            control_omega = Kp * error[1] + Ki * integral_error[1]  # Commande de vitesse angulaire
            self.update_pose(control_v, control_omega, dt)
            reference_trajectory.advance_waypoint()

# Définir la trajectoire de référence
reference_path = np.array([[0, 0], [5, 5], [10, 2], [15, 10]])
trajectory = ReferenceTrajectory(reference_path)

# Initialiser le robot
initial_pose = np.array([0, 0, 0])  # Position et orientation initiales
robot = MobileRobot(initial_pose)

# Paramètres du régulateur PI
Kp = 0.5
Ki = 0.1
dt = 0.1  # Intervalle de temps

# Commande du robot pour suivre la trajectoire
robot.follow_trajectory(trajectory, Kp, Ki, dt)

# Afficher la trajectoire du robot et de la référence
plt.plot(robot.pose[0], robot.pose[1], 'ro', label='Robot')
plt.plot(reference_path[:, 0], reference_path[:, 1], 'b--', label='Reference Trajectory')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Robot Following Reference Trajectory')
plt.legend()
plt.grid(True)
plt.show()
