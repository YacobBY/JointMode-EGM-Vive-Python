from ViveThread import ViveThread
import pprint
import time
import os,sys
import numpy as np


pp = pprint.PrettyPrinter(indent=4)
vive = ViveThread()
vive.start()

try:
    while True:
        d=vive.state
        pose=vive.pose
        if len(d):
            pp.pprint(d)
            print()
            print()
            print(pose)
            time.sleep(1/10)
            sys.stdout.flush()
            os.system('cls')
except KeyboardInterrupt:
    print("Control+C pressed, shutting down...")
    vive.stop()
