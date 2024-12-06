# This is a template code. Please save it in a proper .py file.
from turtle import speed
import rtmaps.types
import numpy as np
import rtmaps.core as rt
import math
import rtmaps.reading_policy
from rtmaps.base_component import BaseComponent  # base class
from simple_pid import PID
from scipy.optimize import curve_fit


# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)  # call base class constructor
        self.pid = PID(0.01, 0.0, 0.0, setpoint=0)
        self.pid.sample_time = 0.01
        self.pid.output_limits = (-1, 1)

    def Dynamic(self):

        self.add_input("Vmax_command", rtmaps.types.ANY)
        self.add_input("obstacle_info", rtmaps.types.ANY)  # Information from the obstacle detection module : Need to stop and/or obstacle avoidance
        self.add_input("Target_pt", rtmaps.types.ANY)
        self.add_input("Virage", rtmaps.types.ANY)  # new
        self.add_input("currentspeed", rtmaps.types.ANY)  # new
        self.add_input("isStop", rtmaps.types.ANY)
        self.add_input("distanceStop", rtmaps.types.ANY)

        self.add_output("speed", rtmaps.types.AUTO)  # Target speed

    # Birth() will be called once at diagram execution startup
    def Birth(self):
        self.outputs["speed"].write(0.)  # Write 0 at the beginning
        print("Python Birth")
        self.fin = False  # At the start, the end is not reached
        self.max_speed = 0.  # TODO
        self.input = [0, 0]
        self.breakval = 0.
        self.is_obstacle = None
        self.speed = 0.
        self.exceptspeed = 0.
        self.virage = 0.
        self.currentspeed = 0.

        self.params = (0, 0)
        self.isStop = 0
        self.distanceStop = 0
        self.x_vitesse = np.array([1056,256])
        self.y_vitesse = np.array([20,0])

    # Core() is called every time you have a new input
    def Core(self):

        # Variables definition
        maneuver_speed = 5.  # Speed to reach while avoiding an obstacles

        try:
            index = self.input_that_answered
        except:
            index = -1
        #Because rtmaps are not entered at the same time, adding the index will prevent failure to enter the
        #but the lastest version of RTMaps correct this bug
        if index == 0:
            self.max_speed = self.inputs["Vmax_command"].ioelt.data  # Speed to reach in a straight line
            self.y_vitesse[0] = self.max_speed * 80/100
            self.params, covariance = curve_fit(lambda x, a, b: a * x + b, self.x_vitesse, self.y_vitesse)
        if index == 1:
            input = self.inputs["obstacle_info"].ioelt  # get info about the obstacle
            #读取刹车值，假如大于0速度就设置为0
            self.breakval = input.data[0]  # Need to stop
            self.is_obstacle = input.data[1]  # Avoidance maneuver

        if index == 3:
            self.virage = self.inputs["Virage"].ioelt.data  # new

        if index == 4:
            self.currentspeed = self.inputs["currentspeed"].ioelt.data
        if index ==5:
            self.isStop = self.inputs["isStop"].ioelt.data
        if index ==6:
            self.distanceStop = self.inputs["distanceStop"].ioelt.data
        if index == 2:
            # x_fit = np.linspace(min(self.x_vitesse), max(self.x_vitesse), 100)
            # y_fit = (lambda x, a, b: a * x + b)(x_fit, *params)

            Target = self.inputs["Target_pt"].ioelt.data  # List of points
            Target = np.array(Target).reshape(-1, 3)

            # X_list = Target[0]
            # Y_list = Target[1]
            pointdevant = Target[Target[:, 0] > 2.5] # 具体作用？
            pointdevant = pointdevant[pointdevant[:, 0] < 7.5]

            if pointdevant.size >= 2:

                # x = pointdevant[0,0]
                # y = pointdevant[0,1]
                # angle = math.atan2(y, x) *15
                # self.speed = self.max_speed /abs(angle)
                # print(abs(angle))

                #Need to stop, then stop
                if self.breakval > 0:
                    # print(self.breakval)
                    # self.speed =0.
                    self.exceptspeed = 0.
                # Else, if there is an avoiding maneuver and the speed if higher than the maneauver speed,
                elif self.virage != 0:
                    self.speed = self.exceptspeed * (80 / 100)
                    #if speed is in bound of 4:10 it will keep it's value
                    #if it's more than 10 speed will be 10
                    #if it's less than 4 speed will be 4
                    self.speed = min(10, max(self.speed,4))
                    # if self.speed <= 8:
                    #         self.speed = 8
                    #self.exceptspeed = 8
                elif self.is_obstacle == 1 and self.speed > maneuver_speed:

                    self.exceptspeed = maneuver_speed
                elif self.isStop == 1:
                    if self.distanceStop >= 400:
                        self.exceptspeed = (lambda x, a, b: a * x + b)(self.distanceStop, *self.params)
                    else :
                        self.exceptspeed = 0
                else:
                    self.exceptspeed = self.max_speed

                # Finally, make sure than the speed will not be higher than the max speed
                if self.speed > self.max_speed:
                    # self.speed = self.max_speed
                    self.exceptspeed = self.max_speed
                if self.speed < self.exceptspeed:
                    command = self.pid(self.speed - self.exceptspeed)
                    self.speed += (command)
                    # Add a limiting condition at startup to limit a startup speed to 0.8 when the real-time speed is equal to zero
                    if self.speed > self.currentspeed and self.currentspeed == 0:
                        self.speed = 0.8
                    self.outputs["speed"].write(self.speed)
                else:
                    command = self.pid(self.speed - self.exceptspeed)
                    self.speed += 8 * (command)
                    self.outputs["speed"].write(self.speed)  # and write the final speed to the output

            if pointdevant.size < 2:
                print("END REACHED")
                self.speed = 0.
                self.outputs["speed"].write(self.speed)

    # Death() will be called once at diagram execution shutdown
    def Death(self):
        pass