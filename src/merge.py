import mujoco
import mujoco.viewer
import numpy as np
import time
import xml.etree.ElementTree as ET

class CarController:
    def __init__(self, model_path, landmarks):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        
        self.forward_actuator_id = self.model.actuator('forward').id
        self.turn_actuator_id = self.model.actuator('turn').id
        
        self.landmarks = landmarks
        self.num_landmarks = self.landmarks.shape[0]
        
        # Noise parameters
        self.motion_noise = 0.1
        self.measurement_noise = 0.1
        
        # Kalman filter initialization
        self.x = np.zeros(3)  # Initial state [x, y, theta]
        self.P = np.eye(3)    # Initial covariance matrix

    def generate_motion(self, v_left, v_right):
        v = (v_left + v_right) / 2
        omega = (v_right - v_left) / self.get_wheel_distance()
        dt = 0.01  # Time step

        dx = v * dt
        dy = 0
        dtheta = omega * dt

        return np.array([dx, dy, dtheta])

    def get_wheel_distance(self):
        left_wheel_body_id = self.model.body('left wheel').id
        right_wheel_body_id = self.model.body('right wheel').id

        left_wheel_pos = self.model.body_pos[left_wheel_body_id]
        right_wheel_pos = self.model.body_pos[right_wheel_body_id]

        wheel_distance = np.linalg.norm(left_wheel_pos[:2] - right_wheel_pos[:2])
        return wheel_distance
    
    def get_wheel_angles(self):
        left_joint_id = self.model.joint('left').id
        right_joint_id = self.model.joint('right').id

        left_wheel_angle = self.data.qpos[left_joint_id]
        right_wheel_angle = self.data.qpos[right_joint_id]

        return left_wheel_angle, right_wheel_angle
    
    def get_wheel_speeds(self):
        left_joint_id = self.model.joint('left').id
        right_joint_id = self.model.joint('right').id

        left_wheel_speed = self.data.qvel[left_joint_id]
        right_wheel_speed = self.data.qvel[right_joint_id]

        return left_wheel_speed, right_wheel_speed
    
    def generate_measurements(self):
        true_distances = np.linalg.norm(self.landmarks - self.x[:2], axis=1)
        measured_distances = true_distances + np.random.normal(0, self.measurement_noise, size=self.num_landmarks)
        measured_bearings = np.arctan2(self.landmarks[:, 1] - self.x[1], self.landmarks[:, 0] - self.x[0]) - self.x[2]
        return np.vstack((measured_distances, measured_bearings)).T

    def update_kalman_filter(self, control, measurements):
        # Prediction step
        self.x[0] += control[0] * np.cos(self.x[2]) - control[1] * np.sin(self.x[2])
        self.x[1] += control[0] * np.sin(self.x[2]) + control[1] * np.cos(self.x[2])
        self.x[2] += control[2]

        F = np.array([[1, 0, -control[0] * np.sin(self.x[2]) - control[1] * np.cos(self.x[2])],
                      [0, 1, control[0] * np.cos(self.x[2]) - control[1] * np.sin(self.x[2])],
                      [0, 0, 1]])
        
        Q = np.eye(3) * self.motion_noise**2
        self.P = F.dot(self.P).dot(F.T) + Q

        # Measurement update step
        H = np.zeros((self.num_landmarks * 2, 3))
        for i in range(self.num_landmarks):
            delta_x = self.landmarks[i, 0] - self.x[0]
            delta_y = self.landmarks[i, 1] - self.x[1]
            q = delta_x**2 + delta_y**2
            H[2*i, :] = [-delta_x / np.sqrt(q), -delta_y / np.sqrt(q), 0]
            H[2*i+1, :] = [delta_y / q, -delta_x / q, -1]

        R = np.eye(self.num_landmarks * 2) * self.measurement_noise**2
        
        y = measurements - np.vstack((np.linalg.norm(self.landmarks - self.x[:2], axis=1),
                                      np.arctan2(self.landmarks[:, 1] - self.x[1], self.landmarks[:, 0] - self.x[0]) - self.x[2])).T

        S = H.dot(self.P).dot(H.T) + R
        K = self.P.dot(H.T).dot(np.linalg.inv(S))
        self.x += K.dot(y.reshape(-1, 1)).flatten()
        self.P = (np.eye(3) - K.dot(H)).dot(self.P)

    def run(self, points):
        self.x[:2] = points[0]
        self.x[2] = 0

        for i in range(len(points) - 1):
            start = points[i]
            end = points[i + 1]
            direction = np.array(end) - np.array(start)
            distance = np.linalg.norm(direction)
            angle = np.arctan2(direction[1], direction[0]) - self.x[2]
            angle = np.arctan2(np.sin(angle), np.cos(angle))
            self.turn(angle, 4)
            self.move_forward(distance, 4)
            lw,rw=self.get_wheel_speeds()
            control = self.generate_motion(lw,rw)
            measurements = self.generate_measurements()
            self.update_kalman_filter(control, measurements)

    def move_forward(self, distance, speed):
        initial_pos = np.copy(self.data.qpos[:2])
        while np.linalg.norm(self.data.qpos[:2] - initial_pos) < distance:
            self.data.ctrl[self.forward_actuator_id] = speed
            self.data.ctrl[self.turn_actuator_id] = 0
            mujoco.mj_step(self.model, self.data)
            self.viewer.sync()
            time.sleep(0.01)
        self.data.ctrl[self.forward_actuator_id] = 0

    def turn(self, angle, speed):
        target_orientation = self.angle_to_quaternion(angle)
        while True:
            self.data.ctrl[self.forward_actuator_id] = 0
            self.data.ctrl[self.turn_actuator_id] = speed if angle > 0 else -speed
            mujoco.mj_step(self.model, self.data)
            self.viewer.sync()
            time.sleep(0.001)
            current_orientation = self.get_robot_orientation()
            angle_difference = self.get_angle_between_quaternions(current_orientation, target_orientation)
            if angle_difference < 0.4:
                break

    def angle_to_quaternion(self, angle):
        qw = np.cos(angle / 2)
        qx = 0
        qy = 0
        qz = np.sin(angle / 2)
        return np.array([qw, qx, qy, qz])

    def get_angle_between_quaternions(self, q1, q2):
        dot_product = np.dot(q1, q2)
        angle = 2 * np.arccos(np.clip(abs(dot_product), 0, 1))
        return angle

    def get_robot_orientation(self):
        robot_body_id = self.model.body('car').id
        robot_orientation = self.data.xquat[robot_body_id]
        return robot_orientation

def update_flag_positions(xml_path, start_pos, end_pos):
        tree = ET.parse(xml_path)
        root = tree.getroot()

        for body in root.findall('worldbody/body'):
            if body.get('name') == 'start_flag':
                body.set('pos', f"{start_pos[0]} {start_pos[1]} 0.2")
            elif body.get('name') == 'end_flag':
                body.set('pos', f"{end_pos[0]} {end_pos[1]} 0.2")

        tree.write(xml_path)
        
if __name__ == "__main__":
    landmarks_2D_position = np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]])
    points = [
        [0, 0],  # Starting point
        [1, 0],  # Move forward 1 meter
        [1, 0.5],  # Turn 90 degrees to the left and move forward 1 meter
        [0, 1]   # Turn 90 degrees to the left and move forward 1 meter
    ]
    # Update the XML file with new positions
    xml_file_path = "../XML/car_amers.xml"
    update_flag_positions(xml_file_path, [points[0][0], points[0][1], 0.2], [points[-1][0], points[-1][1], 0.2])
    car_controller = CarController(xml_file_path, landmarks_2D_position)
    car_controller.run(points)
