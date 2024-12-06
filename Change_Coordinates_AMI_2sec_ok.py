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
    compt = 0
    dist = np.linalg.norm(traj[compt])
    compt +=1
    while compt < traj.shape[0]-1 and dist < dist2sec:
        dist += np.linalg.norm(traj[compt] - traj[compt-1])
        compt+=1
    if dist > dist2sec:
        # delete the point that made the distance exceed the distance to 2 seconds
        dist -= np.linalg.norm(traj[compt] - traj[compt-1])
        compt-=1
    return compt, dist2sec - dist

def create_point_at_2sec(distleft,pt1,pt2):
    x = pt2[0] - pt1[0]
    y = pt2[1] - pt1[1]
    try :
        const = distleft**2 / (1 + (x / y)**2)
        const = math.sqrt(const)
        pt2sec = np.array(((pt1[0] + x * const / y,pt1[1] + const,0.), (0,0,0)))
    except : # if division by 0 ie y==0
        pt2sec = np.array((pt1[0] + distleft,pt1[1],0.))
    return pt2sec

class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)
        self.obstacle = 0

    def Dynamic(self):
        self.add_input("dataRobot", rtmaps.types.AUTO)
        self.add_input("path", rtmaps.types.AUTO)
        self.add_input("reset", rtmaps.types.AUTO)
        self.add_input("speed", rtmaps.types.AUTO) # km/h

        self.add_output("targetX", rtmaps.types.AUTO, buffer_size=150)
        self.add_output("targetY", rtmaps.types.AUTO, buffer_size=150)
        self.add_output("targetXY", rtmaps.types.AUTO, buffer_size=100000)
        self.add_output("trajXY", rtmaps.types.AUTO, buffer_size=1000)

    def Birth(self):
        print("Python Birth")
        self.i = 0
        self.t_obstacle=0
        self.y_bound_decal = 0
        self.pathIsReceive=False
        self.path = ".\\Trajectoires\\traj_ESI_GPS.csv"
        self.dist2sec = 5.

    def Core(self):
        index = self.input_that_answered

        if index == 0:
            # Robot position real
            UTM_X = self.inputs["dataRobot"].ioelt.data[1]
            UTM_Y = self.inputs["dataRobot"].ioelt.data[2]
            yaw = self.inputs["dataRobot"].ioelt.data[0]
            if self.pathIsReceive==True:
                # Trajectory Read
                # trajectory = np.genfromtxt(str(self.path), delimiter=" ", invalid_raise=False)#str(path)
                trajectory = np.loadtxt(self.path, delimiter=" ")#str(path)
                # print(trajectory)
                trajectoryTab = np.array(trajectory).reshape(-1, 3).astype(np.float64)

                # Transition Matrix creation
                H_src = np.eye(4)
                H_src[0:2, 3] = UTM_X, UTM_Y
                rMatrix = Rot.from_euler("ZYX", np.array([(math.pi / 2 - yaw), 0.0, 0.0]), degrees=False).as_matrix()
                H_src[0:3, 0:3] = rMatrix
                H_src = np.linalg.inv(H_src)

                # Passing all points in the car reference
                trajectoryTab = np.transpose(trajectoryTab)
                transMatrix = np.array(([H_src[0,3]], [H_src[1,3]], [H_src[2,3]]))
                rotMatrix = H_src[0:3, 0:3]
                trajectoryTab = rotMatrix.dot(trajectoryTab) + transMatrix
                trajectoryTab = np.transpose(trajectoryTab)

                list_x = trajectoryTab[self.i:,1]
                list_y = -1 * trajectoryTab[self.i:,0]
                XY = np.copy(trajectoryTab[self.i:,:])

                if XY.shape[0] > 1 :
                    # print(XY.shape[0])

                    nb_points, distleft = claculate_nb_points_and_distleft(self.dist2sec, XY)
                    # print("distleft :: " + str(distleft))
                    incr = 0
                    if np.linalg.norm(XY[0]) < 2.5 :
                        # And not at the end, we move forward in the list
                        self.i += 1
                        incr = 1

                    trajxy = np.copy(XY)
                    # print(" nb pts : " + str(nb_points))
                    # Check if index does not exceed the size of the points inside the trajectory
                    if nb_points == list_x.shape[0]-1:
                        # Affectations for the Outputs
                        list_x = list_x[incr: nb_points + 1]
                        list_y = list_y[incr: nb_points + 1]
                        XY = XY[incr: nb_points + 1, :]
                    else : # Add the last point at the end of the 2 seconds
                        point_added = create_point_at_2sec(distleft,XY[nb_points],XY[nb_points+1])
                        # list_x[nb_points + 1] = point_added[0,1]
                        # list_y[nb_points + 1] = -1 * point_added[0,0]
                        XY[nb_points + 1, :] = point_added[0,:]
                        # print("pt added " + str(point_added))
                        # Affectations for the Outputs
                        list_x = list_x[incr: nb_points + 8]
                        list_y = list_y[incr: nb_points + 8]
                        XY = XY[incr : nb_points + 3, :]
                        # print(XY)


                    # output the nb_points_dist next points
                    XY = -1 * XY
                    XY = XY.reshape(-1)

                    self.outputs["targetX"].write(list_x)
                    self.outputs["targetY"].write(list_y)
                    self.outputs["trajXY"].write(XY)

                    trajxy = trajxy[incr + nb_points + 3:, :]
                    trajxy = -1 * trajxy
                    trajxy = trajxy.reshape(-1)
                    self.outputs["targetXY"].write(trajxy)


                else:
                    print("END REACHED")

        elif index == 1:
            try:
                self.path = self.inputs["path"].ioelt.data
            except:
                self.path = ".\\Trajectoires\\traj1.csv"
            print("<<<<<<<<"+self.path)
            self.pathIsReceive=True


        elif index == 2:
            try:
                reset = self.inputs["reset"].ioelt.data
            except:
                reset = -1
            print("<<<<<<<<<"+str(reset))
            if reset == 1:
                self.i=0

        #no idea what it means
        elif index == 3:
            # try:
            self.dist2sec = round(self.inputs["speed"].ioelt.data * 2 / 3.6, 3) + 2.4 # m/s -- rounds at 1mm
            if self.dist2sec < 5. :
                self.dist2sec = 5.
            print("<<<<<<<<< 2 sec : " + str(self.dist2sec))
            # except:
                # self.dist2sec = 0

    # Death() will be called once at diagram execution shutdown
    def Death(self):
        pass
