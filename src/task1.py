import numpy as np
import matplotlib.pyplot as plt

class MobileRobot():
    def __init__(self, landmarks_2D_position, initial_robot_pose, wheel_distance):
        # Paramètres de l'environnement et des amers
        self.landmarks = landmarks_2D_position
        self.num_landmarks = self.landmarks.shape[0]  # Nombre d'amers
        self.initial_robot_pose = initial_robot_pose
        self.wheel_distance = wheel_distance  # Distance entre les roues

    def set_nb_steps(self, num_time_steps=100):
        self.num_time_steps = num_time_steps  # Nombre d'étapes de temps
    
    def set_noises(self, motion_noise=0.1, measurement_noise=0.1):
        self.motion_noise = motion_noise  # Bruit de mouvement
        self.measurement_noise = measurement_noise  # Bruit de mesure

    # Générer les mouvements du robot (simulation d'odométrie)
    def generate_motion(self):
        v_left = 1.0  # Vitesse de la roue gauche
        v_right = 1.1  # Vitesse de la roue droite

        v = (v_left + v_right) / 2
        omega = (v_right - v_left) / self.wheel_distance
        dt = 1  # Intervalle de temps

        dx = v * dt
        dy = 0
        dtheta = omega * dt

        return np.array([dx, dy, dtheta]), v_left, v_right

    # Générer les mesures des amers
    def generate_measurements(self, robot_pose):
        true_distances = np.linalg.norm(self.landmarks - robot_pose[:2], axis=1)
        measured_distances = true_distances  # Pas de bruit ajouté pour simplifier les tests
        measured_bearings = np.arctan2(self.landmarks[:, 1] - robot_pose[1], self.landmarks[:, 0] - robot_pose[0]) - robot_pose[2]
        return np.vstack((measured_distances, measured_bearings)).T

    # Initialiser le filtre de Kalman
    def init_kalman_filter(self):
        # Matrice d'état x contenant les coordonnées du robot (x, y) et son orientation theta
        # Initialisation avec la position initiale du robot
        global x, P
        x = self.initial_robot_pose.copy()  # Initialisation de l'état à la position initiale du robot
        P = np.eye(3)    # Initialisation de la matrice de covariance avec l'identité

    # Mise à jour du filtre de Kalman
    def update_kalman_filter(self, control, measurements):
        global x, P
        # Prédiction de l'état suivant en utilisant les équations de mouvement
        x[0] += control[0] * np.cos(x[2]) - control[1] * np.sin(x[2])
        x[1] += control[0] * np.sin(x[2]) + control[1] * np.cos(x[2])
        x[2] += control[2]

        # Calcul de la matrice Jacobienne de la fonction de transition
        F = np.array([[1, 0, -control[0] * np.sin(x[2]) - control[1] * np.cos(x[2])],
                    [0, 1, control[0] * np.cos(x[2]) - control[1] * np.sin(x[2])],
                    [0, 0, 1]])

        # Prédiction de la covariance de l'état suivant
        Q = np.eye(3) * self.motion_noise**2  # Matrice de covariance du bruit de mouvement
        P = F.dot(P).dot(F.T) + Q

        # Calcul de la matrice Jacobienne de la fonction de mesure
        H = np.zeros((self.num_landmarks * 2, 3))
        for i in range(self.num_landmarks):
            delta_x = self.landmarks[i, 0] - x[0]
            delta_y = self.landmarks[i, 1] - x[1]
            q = delta_x**2 + delta_y**2
            H[2*i, :] = [-delta_x / np.sqrt(q), -delta_y / np.sqrt(q), 0]
            H[2*i+1, :] = [delta_y / q, -delta_x / q, -1]

        # Calcul de la matrice de covariance du bruit de mesure
        R = np.eye(self.num_landmarks * 2) * self.measurement_noise**2

        # Calcul de la résidu de la mesure
        y = measurements - np.vstack((np.linalg.norm(self.landmarks - x[:2], axis=1),
                                np.arctan2(self.landmarks[:, 1] - x[1], self.landmarks[:, 0] - x[0]) - x[2])).T

        # Correction de l'état prédit en utilisant les mesures
        S = H.dot(P).dot(H.T) + R
        K = P.dot(H.T).dot(np.linalg.inv(S))
        x += K.dot(y.reshape(-1, 1)).flatten()

        # Correction de la covariance
        P = (np.eye(3) - K.dot(H)).dot(P)

    # Simulation de la localisation du robot
    def simulate_localization(self):
        # Listes pour stocker les coordonnées estimées du robot et des roues
        estimated_x = []
        estimated_y = []
        left_wheel_x = []
        left_wheel_y = []
        right_wheel_x = []
        right_wheel_y = []

        robot_pose = self.initial_robot_pose.copy()  # Utilisez une copie pour ne pas modifier l'initiale
        self.init_kalman_filter()

        for t in range(self.num_time_steps):
            control, v_left, v_right = self.generate_motion()
            
            # Mise à jour de la position réelle du robot pour la simulation
            robot_pose[0] += control[0] * np.cos(robot_pose[2])
            robot_pose[1] += control[0] * np.sin(robot_pose[2])
            robot_pose[2] += control[2]

            # Calcul des positions des roues
            left_wheel_pose = robot_pose.copy()
            right_wheel_pose = robot_pose.copy()
            left_wheel_pose[0] -= (self.wheel_distance / 2) * np.sin(robot_pose[2])
            left_wheel_pose[1] += (self.wheel_distance / 2) * np.cos(robot_pose[2])
            right_wheel_pose[0] += (self.wheel_distance / 2) * np.sin(robot_pose[2])
            right_wheel_pose[1] -= (self.wheel_distance / 2) * np.cos(robot_pose[2])

            left_wheel_x.append(left_wheel_pose[0])
            left_wheel_y.append(left_wheel_pose[1])
            right_wheel_x.append(right_wheel_pose[0])
            right_wheel_y.append(right_wheel_pose[1])

            # Générer les mesures des amers
            measurements = self.generate_measurements(robot_pose)

            # Mettre à jour le filtre de Kalman avec les nouvelles données
            self.update_kalman_filter(control, measurements)

            # Ajouter la position estimée du robot aux listes
            estimated_x.append(x[0])
            estimated_y.append(x[1])

        # Afficher le graphique à la fin de la simulation
        plt.figure(figsize=[10, 10])
        plt.scatter(estimated_x, estimated_y, label='Estimated Robot Path', c='b', marker='.', s=10)  # 's' définit la taille des points
        plt.scatter(left_wheel_x, left_wheel_y, label='Left Wheel Path', c='g', marker='.', s=10)
        plt.scatter(right_wheel_x, right_wheel_y, label='Right Wheel Path', c='r', marker='.', s=10)
        plt.scatter(self.landmarks[:, 0], self.landmarks[:, 1], c='k', marker='o', label='Landmarks', s=100)  # Taille des landmarks
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.xlim(0, 70)
        plt.ylim(0, 70)
        plt.title('Robot Localization')
        plt.legend()
        plt.grid(True)
        plt.show()

landmarks_2D_position = np.array([[10, 10], [20, 50], [50, 20]])  # Positions des amers (x, y)
initial_robot_pose = np.array([40, 40, np.pi/4])  # Position et orientation initiales du robot (x, y, theta)
wheel_distance = 1.0  # Distance entre les roues
mr = MobileRobot(landmarks_2D_position, initial_robot_pose, wheel_distance)
mr.set_noises()
mr.set_nb_steps(100)
mr.simulate_localization()