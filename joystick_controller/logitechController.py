# Code written by Jason Cabrejos 7/18/24
# python 3.12.1 [FOUND BY RUNNING 'python --version']
# dependencies [FOUND BY RUNNING 'pip freeze']:
#   pygame==2.6.0

import pygame
from pygame.locals import *
import time
from threading import Thread, Lock

class Logitech():
    def __init__(self):
        # controller initiaization
        pygame.init()
        pygame.joystick.init()
        # joysticks = []
        # print(f"Number of joysticks: {pygame.joystick.get_count()}")
        # for i in range(pygame.joystick.get_count()):
        #     joysticks.append(pygame.joystick.Joystick(i))
        #     joysticks[i].init()
        #     print("Detected joystick '",joysticks[i].get_name(),"'")

        # self.__controller = joysticks[0]
        if pygame.joystick.get_count() == 0:
            raise Exception("No joystick found.")
        
        # Initialize Controller (Assumes you only have 1 Controller Connected)
        self.__controller = pygame.joystick.Joystick(0)
        self.__controller.init()

        # Detect the number of axes, trackballs, buttons, and hats/triggers/bumpers
        self.__NumberOfAxes=self.__controller.get_numaxes()
        self.__NumberOfTrackballs=self.__controller.get_numballs()
        self.__NumberOfButtons=self.__controller.get_numbuttons()
        self.__NumberOfHats=self.__controller.get_numhats()

        print("Num of Axes: %d \nNum of Trackballs: %d \nNum of buttons: %d \nNum of hats: %d" % (self.__NumberOfAxes,self.__NumberOfTrackballs,self.__NumberOfButtons,self.__NumberOfHats))
        
        # Initializes blank values for the joystick controls 
        self.__axes=[]
        self.__trackballs=[]
        self.__buttons=[]
        self.__hats=[]

        # variables for threading, locking the thread, and running a background thread to always read from the Controller
        self.__running=True
        self.__lock = Lock()
        self.__thread=Thread(target=self.__backgroundThreadControllerPoller)
        self.__thread.start()


    def __backgroundThreadControllerPoller(self):
        # This function is always running in the background, always getting the latest value from the Controller
        while self.__running:
            pygame.event.pump()
            # self.__lock protects the outputs (self.__axes, self.__trackballs, self.__buttons, self.__hats), from being overwritten if multiple programs are reading from it. https://www.pythontutorial.net/python-concurrency/python-threading-lock/ 
            with self.__lock:
                # Get Axes
                self.__axes = [-self.__controller.get_axis(i) for i in range(self.__NumberOfAxes)]

                # Get Trackballs
                self.__trackballs = [self.__controller.get_ball(i) for i in range(self.__NumberOfTrackballs)]

                # Get Buttons
                self.__buttons = [self.__controller.get_button(i) for i in range(self.__NumberOfButtons)]

                # Get Hats
                self.__hats = [self.__controller.get_hat(i) for i in range(self.__NumberOfHats)]
            
        
    def getLatestInputs(self):
        # Obtains Latest Values from Controller
        return {"axis": self.__axes, "trackball": self.__trackballs, "button":self.__buttons, "hat":self.__hats}
    
    def stopLogitechGracefully(self):
        # Stops the background thread and quits pygame gracefully
        self.__running = False
        self.__thread.join()
        pygame.quit()

if __name__=="__main__":
    #EXAMPLE
    try:
        # Set up the logitech class
        logitechController=Logitech()
        while True:
            print(logitechController.getLatestInputs())

    except KeyboardInterrupt:
        # Ctrl + C to Stop this code example
        logitechController.stopLogitechGracefully()
        print("Program terminated.")
