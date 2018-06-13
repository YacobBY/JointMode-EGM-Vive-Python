from robot_egm import Robot_EGM
import time
import sys,os

egm=Robot_EGM()
os.system('cls')
print('********')
connection_status=egm.connect();
print("EGM connection:  ", connection_status)
print('********')
time.sleep(3)
if (connection_status):
    os.system('cls')
    try:
        while True:
            print("EGM loop:  ", egm.loop())
            print(egm.getSeqno())
            print('********')
            print('Joints value: ')
            print(egm.getJoints())
            egm.getJoints()
            egm.setOffset([0,0.1,0],[1,0,0,0])
            time.sleep(1)
            sys.stdout.flush()
            os.system('cls')
    except KeyboardInterrupt:
        print("Control+C pressed, shutting down...")
