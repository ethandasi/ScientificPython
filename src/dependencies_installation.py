# Links
# Sheet of the team : https://docs.google.com/spreadsheets/d/1csx1-5673ZGjShIuW0cIjO9H5bvS-2AtRh0POMWmr-8/edit#gid=0
# Install Mujoco : https://mujoco.readthedocs.io/en/stable/python.html
# Mojuco-RL informations : https://gymnasium.farama.org/environments/mujoco/pusher/

# Basic dependencies include when you install Python
import time
import os
import sys
import json

# Basic dependencies to install
os.system("pip3 install numpy") # You can comment it when you install the dependency
import numpy as np
os.system("pip3 install matplotlib") # You can comment it when you install the dependency
import matplotlib.pyplot as plt
os.system("pip3 install mujoco") # You can comment it when you install the dependency
import mujoco
import mujoco.viewer

# Basic program to verify that the dependencies are installed
print("Hello world")

time.sleep(1)
print(np.cos(10))

test=[]
for i in range (100):
    test.append(i)
plt.plot(test)
plt.title("It's a test plot")
plt.show()

mujoco.viewer.launch()