import math
import numpy as np
import sys

units = {'KB': 1024, 'MB': 1024**2, 'GB': 1024**3, 'TB': 1024**4, 'PB': 1024**5}

# s0 # s1 # e0 # e1 # w0 # w1 # w2
joint_limits = [3.4032, 3.194, 6.1083, 2.67, 6.117, 3.6647, 6.117]

print("The joint limits for one Baxter arm is: %s" % joint_limits)

def calculatePoints(numOfJoints = 7, resolution = 15, arm = 'right'):
	"""
	Calculate the number of configurations given the grid config.
	"""
	numOfLimbs = 1
	if arm is 'both_arms':
		numOfLimbs = 2
	resolution_rad = resolution * math.pi / 180
	num_points = np.ones(7)
	for i in range(numOfJoints):
		num_points[i] = math.ceil(joint_limits[i]/resolution_rad)
	return np.product(num_points) ** numOfLimbs


def calculateMemory(numberOfPoints = 0, numberOfJoints = 7, name = 'MB'):
	factor = units[name]
	return numberOfPoints * numberOfJoints * 8.0 / factor

if __name__ == '__main__':
	numOfJoints = int(sys.argv[1])
	resolution = int(sys.argv[2])
	arm = None
	if len(sys.argv) > 3:
		arm = sys.argv[3]
	numOfPoints = calculatePoints(numOfJoints, resolution, arm)
	memory = calculateMemory(numOfPoints, numOfJoints)
	print("Number of Joints: %s, Resolution: %s degrees, Limb:%s" % (numOfJoints, resolution, arm))
	print("The number of points is: %s" % (numOfPoints))
	print("The memory to store the dataset is: %.2E%s" % (memory, 'MB'))