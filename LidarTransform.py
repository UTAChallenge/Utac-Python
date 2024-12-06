#This is a template code. Please save it in a proper .py file.
import rtmaps.types
import numpy as np
import rtmaps.core as rt
import math
import rtmaps.reading_policy
from rtmaps.base_component import BaseComponent # base class
from scipy.spatial.transform import Rotation as Rot

# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self) # call base class constructor

    def Dynamic(self):
        self.add_input("list_pt", rtmaps.types.ANY) #Coordinates of the next targeted points
        self.add_output("list_transf", rtmaps.types.AUTO, buffer_size=100000) # Target speed

# Birth() will be called once at diagram execution startup
    def Birth(self):
        print("Lidartranform Birth")

# Core() is called every time you have a new input
    def Core(self):

        list_pt = self.inputs["list_pt"].ioelt.data
        
        list_pt = list_pt.reshape(int(np.size(list_pt)/3),3) #3 shapes matrix : 3D cloud points as X,Y and Z
        
        A = [[1.40], [-0.26], [1]]  #x,y,z ；Adjusting the position of the lidar
        
        raMatrix = Rot.from_euler("ZXY", np.array([0.0, 0.0, 17.5]), degrees=True).as_matrix()  #17.8°，Adjustment of the angle of the lidar (shifted downward by 17.8 degree)
        
        list_pt_trans = np.transpose(list_pt) #Conversion of matrices from 3*1 to 1*3, for the purpose of multiplying rotated matrices

        listtransf = raMatrix.dot(list_pt_trans)+ A #rotation and translation
        
        listtransf = np.transpose(listtransf)#re-transpose

        listtransf = listtransf.reshape(-1)#Finally, the output is changed back to an array

        self.outputs["list_transf"].write(listtransf) # and write the final speed to the output

# Death() will be called once at diagram execution shutdown
    def Death(self):
        pass