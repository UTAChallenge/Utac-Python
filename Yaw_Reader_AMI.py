import rtmaps.types
import numpy as np
import math
import rtmaps.core as rt 
import rtmaps.reading_policy 
from rtmaps.base_component import BaseComponent

#Rotation matrix formula, right hand rule, with the middle finger facing you as x, the thumb to the right, as y, and the index finger up as z:
#Rotate around the z-axis, yaw: [[cosa -sina 0][sina cosa 0][0 0 1]]
#Rotate around x-axis, roll : [[1 0 0][0 cosa -sina][0 sina cosa]]
#Round the y-axis, pitch : [[cosa 0 sina][0 1 0][-sina 0 cosa]]

class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)

    def Dynamic(self):
        self.add_input("in", rtmaps.types.ANY)
        self.add_output("out", rtmaps.types.AUTO)

    def Birth(self):
        print("Python Birth")

    def Core(self):
        out = self.inputs["in"].ioelt.data
        #With the car as the origin of the axes, and the left hand coordinate system, pointing towards ourselves,
        # we only need the angle of the z-axis, adjusted to the direction in which the car is travelling head-on
        out[2] = math.radians(out[2]+180) #Select the z-axis angle of rotation, and here I've tried subtracting 180 degrees and it works
        # print(np.float64(out[2]))
        self.outputs["out"].write(np.float64(out[2]))

    def Death(self):
        pass

