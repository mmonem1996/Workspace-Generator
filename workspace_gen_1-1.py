from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import numpy as np
from math import cos, sin, pi
from itertools import product


class DH_Parameters:
    def __init__(self):
        self.Links = []

    def add_link(self, alpha, a, d, limits:list, revolute=True):
        self.Links.append([a, alpha, d, revolute, limits])

    def forward_kinematics(self, values: list, start_link=0, end_link=None, end_effector_t=None):
        if end_link is None: end_link = len(self.Links) - 1

        if len(values) > len(self.Links): 
            raise(ValueError('num of values > num of links'))

        if (end_link - start_link + 1) != len(values):
        	raise(ValueError('links entered != num of values'))
        T = np.identity(4)
        for i in range(start_link, end_link + 1):
            link = self.Links[i]
            a, alpha, d, revolute, limits = tuple(link[:])
            th = values[i]
            if revolute is True: # If revolute
                Ti = np.array([[           cos(th),           -sin(th),           0,               a],
                               [sin(th)*cos(alpha), cos(th)*cos(alpha), -sin(alpha), -sin(alpha) * d],
                               [sin(th)*sin(alpha), cos(th)*sin(alpha),  cos(alpha),  cos(alpha) * d],
                               [                 0,                  0,           0,               1]
                               ])
            else:               # If prismatic
                Ti = np.array([[           cos(d),           -sin(d),           0,                a],
                               [sin(d)*cos(alpha), cos(d)*cos(alpha), -sin(alpha), -sin(alpha) * th],
                               [sin(d)*sin(alpha), cos(d)*sin(alpha),  cos(alpha),  cos(alpha) * th],
                               [                0,                 0,           0,                1]
                               ])
            T = T @ Ti
        if end_effector_t is not None:
        	Ti = np.array([[1, 0, 0, end_effector_t[0]],
        		[0, 1, 0, end_effector_t[1]],
        		[0, 0, 1, end_effector_t[2]],
        		[0, 0, 0, 1],
        		])
        	T = T @ Ti

        return T

    def generated_values(self, resolutions: list, singular_threshold, end_effector_t=None, dims = None):
        # this function generates values for all joints variables
        # it returns a generator that has a similar behavior to nested for loops
        if len(resolutions) != len(self.Links):
            raise (ValueError('num of values != num of links'))
        ranges = []
        singular_values = []

        for i in range(len(resolutions)):
            link = self.Links[i]
            ranges.append(list(np.linspace(link[4][0], link[4][1], resolutions[i])))

        generated = product(*ranges)
        for value in generated:
        	j_det = np.linalg.det(self.get_jacobian(value, end_effector_t=end_effector_t, dims=dims))
        	if abs(j_det) < singular_threshold or self.any_joint_near_limit(value): 
        		singular_values.append(value)
        return singular_values

    def any_joint_near_limit(self, values: list):
    	for i, value in enumerate(values):
    		if self.Links[i][4][0] == value or self.Links[i][4][1] == value:
    			return True
    	return False
        

    def get_jacobian(self, values: list, start_link=0, end_link=None, end_effector_t=None, dims = None):

    	if dims is None: dims = [1, 1, 1, 0, 0, 0] # default: only x and y
    	T = self.forward_kinematics(values, start_link, end_link, end_effector_t)

    	if end_link is None: end_link = len(self.Links) - 1
    	jacobian = None
    	for i in range(start_link, end_link + 1):
    		Ti = self.forward_kinematics(values[:i+1], start_link, i)
    		row = (T[:3,3] - Ti[:3,3])
    		row = np.hstack((row, Ti[:3,2]))
    		if i == start_link : jacobian = np.array(row)
    		else: jacobian = np.vstack((jacobian, row))
    	
    	jac = None
    	for i, axis in enumerate(dims):
    		if axis == 1:
    			if i == 0 : jac = jacobian[:, i]
    			else: jac = np.vstack((jac, jacobian[:, i]))
    	
    	jacobian = jac
    	return jacobian




disc_robot = DH_Parameters()
# Sample Robot RRR
disc_robot.add_link(0, 0, 0, [0, pi * 2])
disc_robot.add_link(0, 0.5, 0, [0, pi / 2])
X = []
Y = []
Z = []
v = disc_robot.generated_values([50, 50],singular_threshold=0.01, dims = [1, 1, 0, 0, 0, 0], end_effector_t=(0.25, 0, 0))
for value in v:
    T = disc_robot.forward_kinematics(list(value), end_effector_t=(0.25, 0, 0))
    X.append(T[0, 3])
    Y.append(T[1, 3])
    Z.append(T[2, 3])

fig = plt.figure()
ax = fig.add_subplot(111, projection= '3d')
ax.scatter(X, Y, Z, c='r', marker= 'o')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()