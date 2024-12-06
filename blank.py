# This is a template code. Please save it in a proper .py file.
import rtmaps.types
import rtmaps.core as rt
import rtmaps.reading_policy
from rtmaps.base_component import BaseComponent  # base class

# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)  # call base class constructor

    def Dynamic(self):
        
    # Birth() will be called once at diagram execution startup
    def Birth(self):
       

    # Core() is called every time you have a new input
    def Core(self):
       
    # Death() will be called once at diagram execution shutdown
    def Death(self):
       