# Joint Mode EGM Vive for ABB IRB 120

<p align="center">
    <img src="https://github.com/kavehkamali/JointMode-EGM-Vive-Python/blob/master/demo.gif" width="500">
</p>

This code uses OpenVR and EGM for real-time remote control of an ABB 120 robot. 
The package pyopenvr is used to receive position and orientation of an HTC Vive controller. Then a PID controller sends the position and orientation to the ABB robot. We used a python wrapper of EGM in Joint Mode. The python wrapper is made by boost-python.

## Prerequisite steps:

Note: the code only works in Windows OS and Python 3 vc140 64x

1- Install SteamVR

2 - Install pyopenvr: 
``` 
pip install pyopenvr
```
3- Edit the "udp_connection_name" in the RAPID module "EGM_connection.mod".

4 - Copy the RAPID module "EGM_connection.mod" to the ABB robot controller.



## To run the code:

1- Run "EGM_connection.mod" on the ABB robot controler

2- Run SteamVR

3- Run the python code:
```
python main.py
```
