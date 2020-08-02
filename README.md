# eezybotarm-mk2-control-codes
Here you will find he forward and inverse kinemics, control of posiotion with a camara and your mouse of EEZybotarm (link bellow ).The codes arm in written in pyhon wich conect to arduino through the protocol firmata.
In this code the gripper is not taken into account. The location of the end point is considered at the end of the second section of robotic arm. So it is considered to have three DOF. Also because of the construction it has restricted area to wich the end point can be in, the are is shown in a 3d plot in one of the files.

Requirements
-An EEZybotamr controlled 
-Arduino Mega (or just have to specify differently in the codes)
-Upload standart firmata to the arduino
-Install libraries: numpy, cv2, pyfirmata, time, math (almost nothing)

For the inverse and forward finematics I followed a robotic course in youtube by Angela Sodemann.
https://www.youtube.com/channel/UChzNrscXN6F11KeV4EHlHHA

EEZybotarm MK2 desgin by Carlo Franciscone

https://www.thingiverse.com/thing:1454048


