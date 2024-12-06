import compat
import pyfirmata as fir
import time

import rtmaps.types
import rtmaps.reading_policy
from rtmaps.base_component import BaseComponent  # base class
import rtmaps.core as rt

fir.inspect.getargs

class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)
        self.Board = None

    def Dynamic(self):
        self.add_input("cligno_traj", rtmaps.types.AUTO)
        self.add_input("cligno_obstacles", rtmaps.types.AUTO)

    def Birth(self):
        try:
            self.Board = fir.Arduino('COM3')  # Initialisation de la carte Arduino sur le port COM3
            print("Connexion à la carte Arduino réussie.")
            self.Board.digital[8].mode = fir.OUTPUT  # Définition des broches 8 comme sortie
            self.Board.digital[3].mode = fir.OUTPUT  # Définition des broches 3 comme sortie
            print("Configuration des broches réussie.")
        except Exception as e:
            print(f"Erreur de connexion ou de configuration de la carte Arduino: {e}")
            self.Board = None

        # Initialisation des variables de clignotants
        self.cligno_traj = 0  # État du clignotant de trajectoire
        self.cligno_obstacles = 0  # État du clignotant d'obstacles

    def Core(self):
        if not self.Board:
            return

        try:
            self.cligno_traj = self.inputs["cligno_traj"].ioelt.data
        except Exception as e:
            print(f"Erreur de lecture de cligno_traj: {e}")

        try:
            self.cligno_obstacles = self.inputs["cligno_obstacles"].ioelt.data
        except Exception as e:
            print(f"Erreur de lecture de cligno_obstacles: {e}")

        try:
            # Si le clignotant de trajectoire indique DROITE
            if self.cligno_traj == 1 or self.cligno_obstacles == 1:
                self.Board.digital[8].write(1)  # Allumer le clignotant droit
                time.sleep(3)  # Attendre 3 secondes
                self.Board.digital[8].write(0)  # Éteindre le clignotant droit

            # Si le clignotant de trajectoire ou d'obstacles indique GAUCHE
            elif self.cligno_traj == 2 or self.cligno_obstacles == 2:
                self.Board.digital[3].write(1)  # Allumer le clignotant gauche
                time.sleep(3)  # Attendre 3 secondes
                self.Board.digital[3].write(0)  # Éteindre le clignotant gauche
        except Exception as e:
            print(f"Erreur lors de l'écriture sur les broches: {e}")

    def Death(self):
        if self.Board:
            try:
                self.Board.exit()  # Fermer la communication avec l'Arduino
                print("Fermeture de la connexion avec l'Arduino réussie.")
            except Exception as e:
                print(f"Erreur lors de la fermeture de la connexion avec l'Arduino: {e}")
