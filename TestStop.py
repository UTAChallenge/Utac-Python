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
import os
# class ConvNet(nn.Module):
#     def __init__(self, nb_classes):
#         super(ConvNet, self).__init__()
#         self.conv1 = nn.Conv2d(3, 64, kernel_size=(3, 3), stride=(1, 1), padding=(1, 1))
#         self.pool1 = nn.MaxPool2d(kernel_size=(2, 2), stride=(2, 2))
#         self.conv2 = nn.Conv2d(64, 128, kernel_size=(3, 3), stride=(1, 1), padding=(1, 1))
#         self.pool2 = nn.MaxPool2d(kernel_size=(2, 2), stride=(2, 2))
#         self.dropout1 = nn.Dropout(0.5)
#         self.flatten = nn.Flatten()
#         self.fc1 = nn.Linear(128 * int(32 / 4) * int(32 / 4), 50)
#         self.dropout2 = nn.Dropout(0.5)
#         self.fc2 = nn.Linear(50, nb_classes)
 
#     def forward(self, x):
#         x = F.relu(self.conv1(x))
#         x = self.pool1(x)
#         x = F.relu(self.conv2(x))
#         x = self.pool2(x)
#         x = self.dropout1(x)
#         x = self.flatten(x)
#         x = F.relu(self.fc1(x))
#         x = self.dropout2(x)
#         x = self.fc2(x)
#         return x
 
# Python class that will be called from RTMaps.
class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)  # call base class constructor
 
    def Dynamic(self):
        # self.add_output("xVitesse", rtmaps.types.AUTO)  # Target speed
        # self.add_output("yVitesse", rtmaps.types.AUTO)  # Target speed
        # self.add_output("wVitesse", rtmaps.types.AUTO)  # Target speed
        # self.add_output("hVitesse", rtmaps.types.AUTO)  # Target speed
        # self.add_output("distanceVitesse", rtmaps.types.AUTO)  # Target speed
        self.add_output("distanceStop", rtmaps.types.AUTO)  # Target speed
        self.add_output("isStop", rtmaps.types.AUTO)  # Target speed
        self.add_output("fps", rtmaps.types.AUTO)  # Target speed
        self.add_output("averageStop", rtmaps.types.AUTO)  # Target speed
        #self.add_output("processedFrame", rtmaps.types.IPL_IMAGE)
    # Birth() will be called once at diagram execution startup
    def Birth(self):
        # Données
        x_data = np.array([120,110,100,86,68, 56, 51, 45, 38, 33,30,28,27,26,25])
        y_data = np.array([200,220,250,300,400, 500, 600, 700, 800, 900,1000,1100,1200,1300,1400])
        self.params, covariance = curve_fit(lambda x, a, b, c, d, e: a*x**4 + b*x**3 + c*x**2 + d*x + e, x_data, y_data)
        # Générer des points pour la courbe ajustée
        # x_fit = np.linspace(min(x_data), max(x_data), 100)
        # y_fit = (lambda x, a, b, c, d, e: a*x**4 + b*x**3 + c*x**2 + d*x + e)(x_fit, *self.params)
        self.outputs["averageStop"].write(0)
        # Create a pipeline
        self.pipeline = rs.pipeline()
 
        # Configure the pipeline
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
 
        # Start streaming
        # Démarrer le streaming
        try:
            profile = self.pipeline.start(self.config)
            print("Birth: RealSense pipeline started successfully")
        except RuntimeError as e:
            print(f"Birth: RealSense pipeline start error: {e}")
            self.pipeline = None
            return


        # self.stop_cascade = cv2.CascadeClassifier(r'C:\Users\victo\OneDrive\Bureau\PING\Detection vitesse best model\30.xml')
        # Charger le classificateur Haar pour les panneaux STOP
        classifier_path = r'C:\Users\timba\OneDrive - ESIGELEC\Ecole\2A\Projet S8\Ping UTAC 2023-2024\Rendu ami\Parcours_urbain\AMI_With_GPS\data\Stop_classificateur.xml'
        if not os.path.exists(classifier_path):
            raise FileNotFoundError(f"Classifier not found at {classifier_path}")

        self.stop_cascadeStop = cv2.CascadeClassifier(classifier_path)
        if self.stop_cascadeStop.empty():
            raise ValueError(f"Failed to load cascade classifier from {classifier_path}")
        else:
            print(f"Birth: Successfully loaded cascade classifier from {classifier_path}")


# Define a mapping from class numbers to class names
        # self.class_names = {
        #     0: 'Speed limit (20km/h)',
        #     1: 'Speed limit (30km/h)',
        #     2: 'Speed limit (50km/h)',
        #     3: 'Speed limit (60km/h)',
        #     4: 'Speed limit (70km/h)',
        #     5: 'Speed limit (80km/h)',
        #     6: 'Speed limit (100km/h)',
        #     7: 'Speed limit (120km/h)',
           
        #     # ... Add other class names as needed
        # }
 
        # # Load the model
        # self.model_path = r'C:\Users\victo\OneDrive\Bureau\PING\Detection vitesse best model\model_epoch_48.pt'
        # self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        # self.model = ConvNet(nb_classes=8)
        # self.model.load_state_dict(torch.load(self.model_path, map_location=self.device))
        # self.model.to(self.device)
        # self.model.eval()
 
        # Define the desired display width and height
        display_width = 960
        display_height = 540
 
        # Initialize VideoWriter
        output_path = r'C:\Users\timba\OneDrive - ESIGELEC\Ecole\2A\Projet S8\Ping UTAC 2023-2024\Rendu ami\Parcours_urbain\AMI_With_GPS\data\output_video.avi'
        fourcc = cv2.VideoWriter_fourcc(*'XVID')  # You can choose a different codec if needed
        self.output_video = cv2.VideoWriter(output_path, fourcc, 20.0, (display_width, display_height))
        self.histoStop = [0,0,0,0,0,0,0,0,0,0]
        self.compteur = 0

        
        # Transformation for the model input
        # self.transform = transforms.Compose([
        #     transforms.ToPILImage(),
        #     transforms.Resize((32, 32)),
        #     transforms.ToTensor()
        # ])
 
    # Core() is called every time you have a new input
    def Core(self):
        if self.pipeline is None:
            print("Core: RealSense pipeline is not initialized")
            return
        try: 
            start_time = cv2.getTickCount()  # Record the start time
            # Wait for a coherent pair of frames
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                print("Core: No color frame received")
                return
            # Convert color frame to a numpy array
            color_image = np.asanyarray(color_frame.get_data())
            color_image2 = color_image[70:300, 550:]
            cv2.imshow('ROI', color_image2)
            # panneauxVitesse = self.stop_cascade.detectMultiScale(color_image, scaleFactor=1.01, minNeighbors=20, minSize=(20, 20))
            panneauxStop = self.stop_cascadeStop.detectMultiScale(color_image2, scaleFactor=1.01, minNeighbors=20, minSize=(20, 20))
            # detected_signsVitesse = []  # List to store information about detected signs
            detected_signsStop = []  # List to store information about detected signs
            # for (x, y, w, h) in panneauxVitesse:
            #     detected_signsVitesse.append({'x': x, 'y': y, 'width': w, 'height': h})
            # for sign in detected_signsVitesse:
            #     x, y, w, h = sign['x'], sign['y'], sign['width'], sign['height']
            #     cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            #     detected_image = color_image[y:y+h, x:x+w]
            #     # Transform the detected image for the model input
            #     input_tensor = self.transform(detected_image).unsqueeze(0).to(self.device)
            #     with torch.no_grad():
            #         output = self.model(input_tensor)
            #     predicted_class = torch.argmax(output).item()
            #     predicted_class_name = self.class_names.get(predicted_class, f'Class {predicted_class}')
            #     print(f"Panonceau stop détecté à la position ({x}, {y}), taille : {w} x {h}")
            #     print(f'Model Prediction: {predicted_class_name}')
            #     self.outputs["xVitesse"].write(x)
            #     self.outputs["yVitesse"].write(y)
            #     self.outputs["wVitesse"].write(w)
            #     self.outputs["hVitesse"].write(h)
            istop=0
            for (x, y, w, h) in panneauxStop:
                detected_signsStop.append({'x': x, 'y': y, 'width': w, 'height': h})
                print(f"Detected stop sign at x={x}, y={y}, width={w}, height={h}")

            for sign in detected_signsStop:
                x, y, w, h = sign['x'], sign['y'], sign['width'], sign['height']
                if(w<120) :
                    distance = (lambda x, a, b, c, d, e: a*x**4 + b*x**3 + c*x**2 + d*x + e)(w, *self.params)
                else :
                    distance = 200
                cv2.rectangle(color_image2, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(color_image2, f"Dist: {distance:.2f} cm", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # self.outputs["xStop"].write(x)
                # self.outputs["yStop"].write(y)
                # self.outputs["wStop"].write(w)
                # self.outputs["hStop"].write(h)
                self.outputs["distanceStop"].write(distance)
                istop = 1

            if istop == 1 :
                self.histoStop[self.compteur]=1
                self.compteur = (self.compteur + 1)%10
                self.outputs["isStop"].write(1)
            else :
                self.histoStop[self.compteur]=0
                self.compteur = (self.compteur + 1)%10
                self.outputs["isStop"].write(0)
            if (sum(self.histoStop)/len(self.histoStop))>=0.7:
                self.outputs["averageStop"].write(1)
            else :
                self.outputs["averageStop"].write(0)
            end_time = cv2.getTickCount()  # Record the end time
            elapsed_time = (end_time - start_time) / cv2.getTickFrequency()  # Calculate elapsed time in seconds
            fps = 1.0 / elapsed_time  # Calculate frames per second
            self.outputs["fps"].write(fps)
            self.output_video.write(color_image)
    
            cv2.imshow('Real-Time Video', color_image)
            key = cv2.waitKey(1)
            if key == 27:
                rt.sleep(10)
                print("Core: Escape key pressed, exiting")
                self.Stop()

            print(f"Core: Frame processed, FPS: {fps:.2f}")
        except Exception as e:
            print(f"Core: Exception occurred: {e}")


    # Death() will be called once at diagram execution shutdown
    def Death(self):
        # Stop streaming
        try:
            if hasattr(self, 'output_video'):
                self.output_video.release()
            if hasattr(self, 'pipeline') and self.pipeline:
                self.pipeline.stop()
            cv2.destroyAllWindows()
            print("Death: Resources released and windows closed")
        except Exception as e:
            print("Death: Exception occurred: {e}")