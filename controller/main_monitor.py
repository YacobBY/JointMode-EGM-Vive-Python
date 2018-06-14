from controller import control_thread
import pprint
import time
import os,sys
import numpy as np
import traceback

controller=control_thread()
try:
    controller.start()
    while True:
        if(controller.controller_except_flag):
            print("Exception occured in controller ...")
            print(controller.exception)
            traceback.print_tb(controller.exception.__traceback__)
            controller.stop()
            break
        print(controller.J)
        print("controller.Xcv:")
        print(controller.Xcv)
        print("controller.Xcr:")
        print(controller.Xcr)
        print("grip_button_pressed: ")
        print(controller.grip_button_pressed)
        time.sleep(0.1)
        os.system('cls')
except:
    print("Control+C pressed, shutting down...")
    controller.stop()
