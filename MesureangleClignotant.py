import rtmaps.types
import numpy as np
import rtmaps.core as rt
import rtmaps.reading_policy
from rtmaps.base_component import BaseComponent

def unit_vector(vector):
    # Returns the unit vector of the vector.
    return vector / np.linalg.norm(vector)

def a_droite(p1,p2):
    a = p2[0] * p1[1]
    b = p2[1] * p1[0]
    return (a - b) <= 0

def angle_between(v1, v2):
    # Returns the angle in radians between vectors 'v1' and 'v2'
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.degrees(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)))


class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self) # call base class constructor

    def Dynamic(self):
        # Coordinates of the next targeted points
        self.add_input("TargetX", rtmaps.types.ANY)
        self.add_input("TargetY", rtmaps.types.ANY)

        self.add_output("StatusClignotant", rtmaps.types.AUTO) # 0 : tout droit, 1 : Gauche, 2 : Droite
        self.add_property("angleDetection", 20)

    def Birth(self):
        self.outputs["StatusClignotant"].write(0) #Write 0 at the beginning
        self.angleDetection = self.properties["angleDetection"].data
        print("Python Birth")
        self.targetX = np.array(())
        self.targetY = np.array(())

    def Core(self):
        try:
            self.targetX = self.inputs["TargetY"].ioelt.data
        except:
            pass
        try:
            self.targetY = self.inputs["TargetX"].ioelt.data
        except:
            pass
        if self.targetX.shape[0] > 0 and self.targetY.shape[0] > 0 :
            fin = (self.targetX[-1], self.targetY[-1])
            debut = (self.targetX[0], self.targetY[0])
            angle = angle_between(debut, fin)
            
            b = a_droite(debut,fin)
            
            clignotant = 0
            if angle>0.1:
                #print(angle)
                pass
            if angle> self.angleDetection:
                if b :
                    # print("------------------- DROITE")
                    clignotant = 2
                else :
                    # print("------------------- GAUCHE")
                    clignotant = 1
                # print("----------------------------------------------------------------------------------------------Active----------------------------------------------------------------------------------------------------------")
            
            self.outputs["StatusClignotant"].write(clignotant)
    


    def Death(self):
        pass