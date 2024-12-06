import rtmaps.types
import numpy as np
import rtmaps.core as rt
import rtmaps.reading_policy
import utm
import math
import time
from rtmaps.base_component import BaseComponent


class rtmaps_python(BaseComponent):
    def __init__(self): #Executed once every time rtmaps is opened
        BaseComponent.__init__(self)

        self.previousY = 0.0
        self.previousX = 0.0
        self.trajectoryFile = None
        self.distPts = 2

        self.IMU = None
        self.recording = 0


    def Dynamic(self):
        self.add_input("in", rtmaps.types.ANY)
        self.add_input("recording", rtmaps.types.ANY)

    def Birth(self):#Once per execution of the entire project procedure
        print("Python Birth")
        self.trajectoryFile = open("C:/Users/timba/Documents/trajutacParkAssist2024.csv", "w+") #Create a csv file and overwrite it if it's already there
        self.trajectoryFile.close()

    def Core(self):
        self.IMU = self.inputs["in"].ioelt.data
        self.recording = self.inputs["recording"].ioelt.data
        if self.recording == 1:  # if we want to record
            print('okkkkkkkk')
            if not self.trajectoryFile.closed:
                pass
            else:  # if record file is closed we open it
                self.trajectoryFile = open("C:/Users/timba/Documents/trajutacParkAssist2024.csv", "w+")

            # UTM Conversion
            UTM_conv = utm.from_latlon(self.IMU[0], self.IMU[1])
            x = UTM_conv[0]
            y = UTM_conv[1]

            # If the point is further than self.distPts meters from the previous one,we save it
            if math.sqrt((x - self.previousX) ** 2 + (y - self.previousY) ** 2) > self.distPts:

                self.trajectoryFile.write(str(x))
                self.previousX = x
                self.trajectoryFile.write(" ")
                self.trajectoryFile.write(str(y))
                self.previousY = y
                self.trajectoryFile.write(" ")
                self.trajectoryFile.write("0.0")
                self.trajectoryFile.write("\n")
        else:
            if not self.trajectoryFile.closed:
                self.trajectoryFile.close()

    def Death(self):
        if self.recording == 1:

            # record final position (twice bc assay issues)
            UTM_conv = utm.from_latlon(self.IMU[0], self.IMU[1])
            x = UTM_conv[0]
            y = UTM_conv[1]
            self.trajectoryFile.write(str(x))
            self.trajectoryFile.write(" ")
            self.trajectoryFile.write(str(y))
            self.trajectoryFile.write(" ")
            self.trajectoryFile.write("0.0")
            self.trajectoryFile.write("\n")

            self.trajectoryFile.write(str(x))
            self.trajectoryFile.write(" ")
            self.trajectoryFile.write(str(y))
            self.trajectoryFile.write(" ")
            self.trajectoryFile.write("0.0")
            self.trajectoryFile.write("\n")
            self.trajectoryFile.close()
        else:
            self.trajectoryFile.close()
        pass
