# Joint Mode EGM Vive for ABB IRB 120

<p align="center">
    <img src="https://github.com/kavehkamali/RobotPath/blob/master/demo.gif" width="500">
</p>

This code uses OpenVR and EGM for real-time remote control of an ABB 120 robot. 
The package pyopenvr is used to receive position and orientation of an HTC Vive controller. Then a PID controller sends the position and orientation to the ABB robot. We used a python wrapper of EGM in Joint Mode. The python wrapper is made by boost-python.

## Prequisits:

``` 
pip install pyopenvr
```

## To run the code:

```
python main.py
```
