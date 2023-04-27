# match_mocap
scripts to use the Qualisys Motion Capture System at match

## 1. Installation
You need to install pip3
```
sudo apt install pip3
```

Then install the qtm package from Qualisys
```
pip3 install qtm
```

## deviation_receiver.py

Script to get the deviation between the origin of the coordinate system and a averaged centerpoint of the rotation.  The mir platform will perform a rotation around its z axis and calculates the radius of the circle. The radius is  approximately the deviation. 

## qualisys_receiver.py
Script to receive data from the Qualisys motion capture system and publish it to a ROS Topic. This will publish all points of a rigid body and not only the coordinate system. First argument should be the id of the rigid body. If no argument is given, all rigid bodies are printed.