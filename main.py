# Basic dependencies include when you install Python
import time
import os
import sys

# Basic dependencies to install
os.system("pip3 install numpy") # You can comment it when you install the dependency
import numpy as np
os.system("pip3 install matplotlib") # You can comment it when you install the dependency
import matplotlib.pyplot as plt

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