import time
import openvr
import sys
import os
import numpy as np
import threading

"""
Get the HTC Vive controllers keypresses and print them to screen.
You need to do:
export LD_LIBRARY_PATH=$HOME/.steam/steam/steamapps/common/SteamVR/bin/linux64:$HOME/.steam/steam/steamapps/common/tools/bin/linux64:$LD_LIBRARY_PATH
before executing it.
Expected output:
Left controller:
{   'grip_button': False,
    'menu_button': False,
    'trackpad_pressed': False,
    'trackpad_touched': True,
    'trackpad_x': 0.032959990203380585,
    'trackpad_y': -0.8471328020095825,
    'trigger': 0.0,
    'ulButtonPressed': 0L,
    'ulButtonTouched': 4294967296L,
    'unPacketNum': 2037L}
Right controller:
{   'grip_button': False,
    'menu_button': False,
    'trackpad_pressed': False,
    'trackpad_touched': False,
    'trackpad_x': 0.0,
    'trackpad_y': 0.0,
    'trigger': 0.0,
    'ulButtonPressed': 0L,
    'ulButtonTouched': 0L,
    'unPacketNum': 1146L}
Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""
class Vive_thread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self._stop_event = threading.Event()
        self.state={}
        self.pose=np.zeros((3,4))
        self.vive_run_flag=True

    def get_controller_ids(self,vrsys=None):
        if vrsys is None:
            vrsys = openvr.VRSystem()
        else:
            vrsys = vrsys
        #left = None
        right = None
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            device_class = vrsys.getTrackedDeviceClass(i)
            if device_class == openvr.TrackedDeviceClass_Controller:
                role = vrsys.getControllerRoleForTrackedDeviceIndex(i)
                if role == openvr.TrackedControllerRole_RightHand:
                    right = i
                #if role == openvr.TrackedControllerRole_LeftHand:
                #    left = i
        #return left, right
        return right

    def from_controller_state_to_dict(self,pControllerState):
        # docs: https://github.com/ValveSoftware/openvr/wiki/IVRSystem::GetControllerState
        d = {}
        d['unPacketNum'] = pControllerState.unPacketNum
        # on trigger .y is always 0.0 says the docs
        d['trigger'] = pControllerState.rAxis[1].x
        # 0.0 on trigger is fully released
        # -1.0 to 1.0 on joystick and trackpads
        d['trackpad_x'] = pControllerState.rAxis[0].x
        d['trackpad_y'] = pControllerState.rAxis[0].y
        # These are published and always 0.0
        # for i in range(2, 5):
        #     d['unknowns_' + str(i) + '_x'] = pControllerState.rAxis[i].x
        #     d['unknowns_' + str(i) + '_y'] = pControllerState.rAxis[i].y
        d['ulButtonPressed'] = pControllerState.ulButtonPressed
        d['ulButtonTouched'] = pControllerState.ulButtonTouched
        # To make easier to understand what is going on
        # Second bit marks menu button
        d['menu_button'] = bool(pControllerState.ulButtonPressed >> 1 & 1)
        # 32 bit marks trackpad
        d['trackpad_pressed'] = bool(pControllerState.ulButtonPressed >> 32 & 1)
        d['trackpad_touched'] = bool(pControllerState.ulButtonTouched >> 32 & 1)
        # third bit marks grip button
        d['grip_button'] = bool(pControllerState.ulButtonPressed >> 2 & 1)
        # System button can't be read, if you press it
        # the controllers stop reporting
        return d

    def connect(self):
        max_init_retries = 4
        retries = 0
        print("===========================")
        print("Initializing OpenVR...")
        while retries < max_init_retries:
            try:
                openvr.init(openvr.VRApplication_Scene)
                break
            except openvr.OpenVRError as e:
                print("Error when initializing OpenVR (try {} / {})".format(
                      retries + 1, max_init_retries))
                print(e)
                retries += 1
                time.sleep(2.0)
        else:
            print("Could not initialize OpenVR, aborting.")
            print("Make sure the system is correctly plugged, you can also try")
            print("to do:")
            print("killall -9 vrcompositor vrmonitor vrdashboard")
            print("Before running this program again.")
            exit(0)

        print("Success!")
        print("===========================")
        self.vrsystem = openvr.VRSystem()
        poses_t = openvr.TrackedDevicePose_t * openvr.k_unMaxTrackedDeviceCount
        self.poses = poses_t()

        #left_id, right_id = None, None
        self.right_id = None
        print("===========================")
        print("Waiting for controllers...")

        try:
            while self.right_id is None:
            #while left_id is None or right_id is None:
                #left_id, right_id = self.get_controller_ids(vrsystem)
                self.right_id = self.get_controller_ids(self.vrsystem)
                #if (left_id or self.left_controller) and (right_id or self.right_controller):
                if self.right_id:
                    break
                #print("Waiting for controllers...")
                print("Waiting for the right controller...")
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("Control+C pressed, shutting down...")
            openvr.shutdown()

        #print("Left controller ID: " + str(left_id))
        print("Right controller ID: " + str(self.right_id))
        print("===========================")

        #pp = pprint.PrettyPrinter(indent=4)

        reading_rate_hz = 100
        #show_only_new_events = True
        #last_unPacketNum_left = 0
        last_unPacketNum_right = 0

    def run(self):
        while self.vive_run_flag:
            openvr.VRCompositor().waitGetPoses(self.poses, len(self.poses), None, 0)
            #if left_id:
            #    left_pose = poses[left_id]
            #    if self.show:
            #        print(left_pose.mDeviceToAbsoluteTracking)
            #        result, pControllerState = vrsystem.getControllerState(left_id)
            #        d = self.from_controller_state_to_dict(pControllerState)
            #        #if show_only_new_events and last_unPacketNum_left != d['unPacketNum']:
            #        last_unPacketNum_left = d['unPacketNum']
            #        print("Left controller:")
            #        pp.pprint(d)
            #if left_id and right_id and self.show:
            #    print()
            #    print()

            if self.right_id:
                right_pose = self.poses[self.right_id]
                result, pControllerState = self.vrsystem.getControllerState(self.right_id)
                d = self.from_controller_state_to_dict(pControllerState)
                #if show_only_new_events and last_unPacketNum_right != d['unPacketNum']:
                last_unPacketNum_right = d['unPacketNum']
                self.state=d
                temp=right_pose.mDeviceToAbsoluteTracking
                self.pose[0,0:4]=temp[0][0:4]
                self.pose[1,0:4]=temp[1][0:4]
                self.pose[2,0:4]=temp[2][0:4]
                #print("Right controller:")
                #pp.pprint(d)
                #print("right controller pose: ")
                #print(right_pose.mDeviceToAbsoluteTracking)

    def stop(self):
        self.vive_run_flag=False

    def getPose(self):
        return self.pose

    def getState(self):
        return self.state
