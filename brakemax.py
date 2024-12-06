# This is a template code. Please save it in a proper .py file.
import rtmaps.types
import rtmaps.core as rt
import rtmaps.reading_policy
from rtmaps.base_component import BaseComponent  # base class
import pyrealsense2 as rs
import numpy as np
import cv2
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.transforms as transforms
import matplotlib.pyplot as plt
import random
from scipy.optimize import curve_fit
 
# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)  # call base class constructor
 
    def Dynamic(self):
        self.add_input("brakeObs", rtmaps.types.ANY)
        self.add_input("brakeStop", rtmaps.types.ANY)
        self.add_output("brake", rtmaps.types.AUTO)  # Target speed
        #self.add_output("processedFrame", rtmaps.types.IPL_IMAGE)
    # Birth() will be called once at diagram execution startup
    def Birth(self):
        # Données
        self.brakeObs =0.0
        self.brakeStop = 0.0
        self.brake = 0.0
 
    # Core() is called every time you have a new input
    def Core(self):
        self.brakeObs =self.inputs["brakeObs"].ioelt.data
        try:
            self.brakeStop = self.inputs["brakeStop"].ioelt.data
        except Exception as e:
            print("Error:", e)

        self.brake = max(self.brakeObs,self.brakeStop)
        print(self.brake)
        self.outputs["brake"].write(self.brake)
   
    # Death() will be called once at diagram execution shutdown
    def Death(self):
        pass