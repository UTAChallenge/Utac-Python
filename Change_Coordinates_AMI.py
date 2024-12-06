import math
import time
import rtmaps.types
import numpy as np
import rtmaps.core as rt
import rtmaps.reading_policy
from scipy.spatial.transform import Rotation as Rot
from rtmaps.base_component import BaseComponent
import os
import sys


def claculate_nb_points_and_distleft(dist2sec, traj):
    # In the following application, the input is dist2sec:distance of the second point
    # as a scalar5 traj:path of the remaining points in local coordinates
    compt = 0
    dist = np.linalg.norm(traj[compt]) #Paradigms, scalars, are used to define the magnitude of vectors
    compt += 1
    #Condition 1: When compt is less than the remaining path minus one to avoid index crossing error
    #Condition 2: Loop when the distance is less than the distance to reach the second point
    while compt < traj.shape[0] - 1 and dist < dist2sec:
        dist += np.linalg.norm(traj[compt] - traj[compt - 1])
        compt += 1

    if dist > dist2sec:
        # delete the point that made the distance exceed the distance to 2 seconds
        dist -= np.linalg.norm(traj[compt] - traj[compt - 1])
        compt -= 1
    return compt, dist2sec - dist


def create_point_at_2sec(distleft, pt1, pt2):
    x = pt2[0] - pt1[0]
    y = pt2[1] - pt1[1]
    try:
        const = distleft ** 2 / (1 + (x / y) ** 2)
        const = math.sqrt(const)
        pt2sec = np.array(((pt1[0] + x * const / y, pt1[1] + const, 0.), (0, 0, 0)))
    except:  # if division by 0 ie y==0
        pt2sec = np.array((pt1[0] + distleft, pt1[1], 0.))
    return pt2sec


class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)
        self.obstacle = 0

    def Dynamic(self):
        self.add_input("dataRobot", rtmaps.types.AUTO)

        self.add_output("targetX", rtmaps.types.AUTO, buffer_size=150)
        self.add_output("targetY", rtmaps.types.AUTO, buffer_size=150)
        self.add_output("targetXY", rtmaps.types.AUTO, buffer_size=100000)
        self.add_output("trajXY", rtmaps.types.AUTO, buffer_size=1000)

    def Birth(self):
        print("Python Birth")
        self.i = 0
        #self.path = r"C:/Users/timba/Documents/trajutacParkAssist2024.csv""
        self.path = r"C:\Users\max\OneDrive\Documents\ping\2023-2024 S8\Parcours_urbain\AMI_With_GPS\data\Parking_esigelec\traj_ESI_GPS.csv"
        self.dist2sec = 5.  #Changing the size increases the point of the guide line

    def Core(self):

        # Robot position real
        UTM_X = self.inputs["dataRobot"].ioelt.data[1] #Longitude in UTM
        UTM_Y = self.inputs["dataRobot"].ioelt.data[2] #Latitude in UTM
        yaw = self.inputs["dataRobot"].ioelt.data[0] #z-rotation axis with the car as the axis, right-handed coordinates

        # Trajectory Read
        # trajectory = np.genfromtxt(str(self.path), delimiter=" ", invalid_raise=False) #This read function is generally targeted at importing large data sets
        trajectory = np.loadtxt(self.path, delimiter=" ")  # str(path) Reads the entire CSV file, separated by spaces
        #Personally, I think this could be optimised here, as the need to read all the csv information each time can be memory and computing intensive
        #Because trajectory is three parameters, longitude, latitude and altitude, but the height is zero by default, and then force the format to double floating point
        trajectoryTab = np.array(trajectory).reshape(-1, 3).astype(np.float64)

        # Transition Matrix creation
        H_src = np.eye(4) #Create a 4*4 diagonal matrix with a diagonal of 1
        #In [[0:3][0:3]], the parameter represents the rotation of the coordinates, [[0:2], 3] represents the XYZ, and the last one [3, 3] is no-sens

        H_src[0:2, 3] = UTM_X, UTM_Y

        # Create a rotation matrix, each time the path point is updated, the vehicle turns the corresponding angle.
        # The internal equation means to take the input yaw angle and make it negative and add 90° to it,
        # the result is that if the original angle was origin to destination, the end point will be origin after the calculation.
        rMatrix = Rot.from_euler("ZYX", np.array([(math.pi / 2 - yaw), 0.0, 0.0]), degrees=False).as_matrix()
        H_src[0:3, 0:3] = rMatrix #Add the rotation angle vector to
        H_src = np.linalg.inv(H_src) #inverse matrix

        #The following is the conversion from UTM to the local coordinate system.
        # Passing all points in the car reference
        trajectoryTab = np.transpose(trajectoryTab) #Conversion, which can then be transposed and rotated

        transMatrix = np.array(([H_src[0, 3]], [H_src[1, 3]], [H_src[2, 3]]))
        rotMatrix = H_src[0:3, 0:3]
        trajectoryTab = rotMatrix.dot(trajectoryTab)
        trajectoryTab = trajectoryTab + transMatrix
        trajectoryTab = np.transpose(trajectoryTab)

        #Extracts data on the x, y, and z (default equal to 0) axes for calculating turns
        list_x = trajectoryTab[self.i:, 1]
        list_y = -1 * trajectoryTab[self.i:, 0] #Right-handed coordinate system, y-axis needs a negative sign
        XY = np.copy(trajectoryTab[self.i:, :])

        if XY.shape[0] > 1:
            nb_points, distleft = claculate_nb_points_and_distleft(self.dist2sec, XY)
            incr = 0

            if np.linalg.norm(XY[0]) < 5:
                # And not at the end, we move forward in the list
                self.i += 1
                incr = 1

            trajxy = np.copy(XY)

            # Check if index does not exceed the size of the points inside the trajectory
            if nb_points == list_x.shape[0] - 1:
                # Affectations for the Outputs
                list_x = list_x[incr: nb_points + 1]
                list_y = list_y[incr: nb_points + 1]
                XY = XY[incr: nb_points + 1, :]
            else:  # Add the last point at the end of the 2 seconds
                point_added = create_point_at_2sec(distleft, XY[nb_points], XY[nb_points + 1])
                # list_x[nb_points + 1] = point_added[0,1]
                # list_y[nb_points + 1] = -1 * point_added[0,0]
                XY[nb_points + 1, :] = point_added[0, :]
                # print("pt added " + str(point_added))
                # Affectations for the Outputs
                list_x = list_x[incr: nb_points + 8]
                list_y = list_y[incr: nb_points + 8]
                XY = XY[incr: nb_points + 6, :]
                # print(XY)

            # output the nb_points_dist next points
            XY = -1 * XY
            XY = XY.reshape(-1)

            self.outputs["targetX"].write(list_x)
            self.outputs["targetY"].write(list_y)
            self.outputs["trajXY"].write(XY)

            #The difference between the two trajXY is that the y-axis is pointing differently,
            # as well as the following which will fade the path points earlier with the guide line (with incur set)
            trajxy = trajxy[incr + nb_points + 3:, :]
            trajxy = -1 * trajxy
            trajxy = trajxy.reshape(-1)

            # print(incr)
            # trajxy = -1 * trajxy
            # trajxy = trajxy.reshape(-1)
            #if trajxy.shape[0] > 0:
            self.outputs["targetXY"].write(trajxy)
            #else:
                #print("No target.")
# Death() will be called once at diagram execution shutdown
def Death(self):
    pass
