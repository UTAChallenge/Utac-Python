#This is a template code. Please save it in a proper .py file.
import rtmaps.types
import numpy as np
import rtmaps.core as rt 
import rtmaps.reading_policy 
from rtmaps.base_component import BaseComponent # base class 
import glob
import time

# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self) # call base class constructor

    def Dynamic(self):
        self.add_input("previous", rtmaps.types.ANY) # define input
        self.add_input("next", rtmaps.types.ANY) # define input
        self.add_input("reset", rtmaps.types.ANY)

        self.add_output("path", rtmaps.types.AUTO) # define output
        self.add_output("out_reset", rtmaps.types.AUTO) # define output
        self.add_output("selectedTraj", rtmaps.types.AUTO) # define output
       

# Birth() will be called once at diagram execution startup
    def Birth(self):
        print("Python Birth")
        self.count = 0
        self.listTraj = glob.glob("C:\\Users\\asus\\Desktop\\AMI Project\\Projet PING RTMaps\\AMI_Project\\Trajectoires\\*")
        self.listTraj.sort()
        #for i in self.listTraj:
            #print ("<<<<<<<>>>>>>>>"+i)
        self.outputs["path"].write(self.listTraj[self.count])
        self.outputs["selectedTraj"].write("trajectory "+str(self.count+1))
        self.outputs["out_reset"].write(0)



# Core() is called every time you have a new input
    def Core(self):
        try:
            index = self.input_that_answered
        except:
            index = -1

        if index == 0:
            if self.count == 0:
                self.count=len(self.listTraj)-1
            else:
                self.count-=1
            self.outputs["path"].write(self.listTraj[self.count])
            self.outputs["selectedTraj"].write("trajectory "+str(self.count+1))
            self.outputs["out_reset"].write(0)

        elif index == 1:
            if self.count == len(self.listTraj)-1:
                self.count = 0
            else:
                self.count += 1
            self.outputs["path"].write(self.listTraj[self.count])
            self.outputs["selectedTraj"].write("trajectory "+str(self.count+1))
            self.outputs["out_reset"].write(0)

        elif index == 2:
            self.outputs["out_reset"].write(1)
            time.sleep(1)
            self.outputs["out_reset"].write(0)
        
            
# Death() will be called once at diagram execution shutdown
    def Death(self):
        pass
