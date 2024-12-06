# This is a template code. Please save it in a proper .py file.
import rtmaps.types
import rtmaps.core as rt
import rtmaps.reading_policy
from rtmaps.base_component import BaseComponent  # base class
import pyrealsense2 as rs
import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)  # call base class constructor

    def Dynamic(self):
        self.add_output("x", rtmaps.types.AUTO)  # Target speed
        self.add_output("y", rtmaps.types.AUTO)  # Target speed
        self.add_output("w", rtmaps.types.AUTO)  # Target speed
        self.add_output("h", rtmaps.types.AUTO)  # Target speed
        self.add_output("distance", rtmaps.types.AUTO)  # Target speed
        #self.add_output("processedFrame", rtmaps.types.IPL_IMAGE)
    # Birth() will be called once at diagram execution startup
    def Birth(self):
        # Données
        self.x_data = np.array([340, 280, 250, 115, 78, 67])
        self.y_data = np.array([14, 17, 20, 40, 60, 72])

        # Create a pipeline
        self.pipeline = rs.pipeline()

        # Configure the pipeline
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(self.config)
        self.stop_cascade = cv2.CascadeClassifier(r'C:\Users\timba\OneDrive - ESIGELEC\Ecole\2A\Projet S8\Ping UTAC 2023-2024\Rendu ami\Parcours_urbain\AMI_With_GPS\data\Stop_classificateur.xml')

        # Define the desired display width and height
        display_width = 640
        display_height = 480

        # Initialize VideoWriter
        output_path = r'C:\Users\timba\OneDrive - ESIGELEC\Ecole\2A\Projet S8\Ping UTAC 2023-2024\Rendu ami\Parcours_urbain\AMI_With_GPS\data\output_video.avi'
        fourcc = cv2.VideoWriter_fourcc(*'XVID')  # You can choose a different codec if needed
        self.output_video = cv2.VideoWriter(output_path, fourcc, 20.0, (display_width, display_height))

    # Core() is called every time you have a new input
    def Core(self):
        # Wait for a coherent pair of frames
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # Ajuster la fonction polynomiale
        params, covariance = curve_fit(lambda x, a, b, c, d, e: a*x**4 + b*x**3 + c*x**2 + d*x + e, self.x_data, self.y_data)
        # Générer des points pour la courbe ajustée
        x_fit = np.linspace(min(self.x_data), max(self.x_data), 100)
        y_fit = (lambda x, a, b, c, d, e: a*x**4 + b*x**3 + c*x**2 + d*x + e)(x_fit, *params)

        if not depth_frame or not color_frame:
            pass
        # Convert depth frame to a numpy array
        depth_image = np.asanyarray(depth_frame.get_data())

        # Convert color frame to a numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on the depth image (optional)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # Stack both images horizontally
        images = np.hstack((color_image, depth_colormap))

        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        panneaux = self.stop_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=35, minSize=(55, 55))

        detected_signs = []  # List to store information about detected signs

        for (x, y, w, h) in panneaux:
            detected_signs.append({'x': x, 'y': y, 'width': w, 'height': h})
        for sign in detected_signs:
            x, y, w, h = sign['x'], sign['y'], sign['width'], sign['height']
            distance = (lambda x, a, b, c, d, e: a*x**4 + b*x**3 + c*x**2 + d*x + e)(w, *params)
            cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            self.outputs["x"].write(x)
            self.outputs["y"].write(y)
            self.outputs["w"].write(w)
            self.outputs["h"].write(h)
            self.outputs["distance"].write(distance)
        #self.outputs["processedFrame"].write(color_image)
        self.output_video.write(color_image)
    
    # Death() will be called once at diagram execution shutdown
    def Death(self):
        # Stop streaming
        self.output_video.release()
        self.pipeline.stop()

        # Close all OpenCV windows
        #cv2.destroyAllWindows()