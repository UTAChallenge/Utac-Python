#This is a template code. Please save it in a proper .py file.
import rtmaps.types
import numpy as np
import rtmaps.core as rt 
import rtmaps.reading_policy 
from rtmaps.base_component import BaseComponent # base class 


# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self) # call base class constructor

    def Dynamic(self):
        self.add_input("button0", rtmaps.types.ANY) # define input
        self.add_input("button10", rtmaps.types.ANY) # define input
        self.add_input("button15", rtmaps.types.ANY) # define input
        self.add_input("button20", rtmaps.types.ANY) # define input

        self.add_output("speed_command", rtmaps.types.AUTO) # define output
        self.add_output("led_viewer", rtmaps.types.AUTO)

# Birth() will be called once at diagram execution startup
    def Birth(self):
        print("Python Birth")
        self.lastIndex = 0


# Core() is called every time you have a new input
    def Core(self):
        try:
            index = self.input_that_answered
        except:
            index = 0
        

        if index != self.lastIndex:
            if index == 0 : 
                self.outputs["speed_command"].write(0.)
                self.outputs["led_viewer"].write([1,0,0,0])
            if index == 1 :
                self.outputs["speed_command"].write(10.)
                self.outputs["led_viewer"].write([0,1,0,0])
            if index == 2 :
                self.outputs["speed_command"].write(20.)
                self.outputs["led_viewer"].write([0,0,1,0])
            if index == 3 :
                self.outputs["speed_command"].write(30.)
                self.outputs["led_viewer"].write([0,0,0,1])
        self.lastIndex=index


# Death() will be called once at diagram execution shutdown
    def Death(self):
        pass
