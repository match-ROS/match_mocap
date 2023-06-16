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

## transformation_mocap.py
Script to approximate the transformation betweet the Qualisys-Mocap and the MIR coordinate system. The base and orientation of the Qualisys coordinate system in MIR-coordinates is unknown. The script continuously calculates a transformation with three points in Qualisys- and MIR-coordinates and checks it for accuracy by comparing the transformated Qualisys poses to the real MIR coordinates.

### - Mathematical background
It is assumed that the z-axis of both coordinate systems has the same direction. Moreover, the translational z component in the transformation matrix should be zero. For this case, the script should find the x and y MIR-coordinates from the base of the Qualisys coordinate system and the corresponding z-rotation phi. 

The x and y coordinates can be calculated by defining three circles. The center of the circles are the MIR coordinates and the radius is the value of the qualisys coordinates. With this all three circles should intersect in one point which is the basepoint of the qualisys coordinate system in MIR coordinates. 

The z-rotation phi can now be approximated.  Since we are looking for a transformation matrix:

![](https://latex.codecogs.com/png.latex?%5Cdpi%7B100%7D%20%5Cbg_white%20%5Clarge%20%5E%7BQ%7D%20T%20_%7BM%7D%20%5Ccdot%20_%7B%28Q%29%7Dr%20%3D%20_%7B%28M%29%7Dr)

These formulas can be derived from this:

![](https://latex.codecogs.com/png.latex?%5Cdpi%7B100%7D%20%5Cbg_white%20%5Clarge%20_%7B%28M%29%7Dx%3D_%7B%28Q%29%7Dx%5Ccdot%20cos%28%5Cvarphi%20%29%20-%20_%7B%28Q%29%7Dy*sin%28%5Cvarphi%29&plus;_%7B%28M%29%7Dx_%7BQualisys%2Cbase%7D)
![](https://latex.codecogs.com/png.latex?%5Cdpi%7B100%7D%20%5Cbg_white%20%5Clarge%20_%7B%28M%29%7Dy%3D_%7B%28Q%29%7Dx%5Ccdot%20sin%28%5Cvarphi%20%29%20&plus;%20_%7B%28Q%29%7Dy*cos%28%5Cvarphi%29&plus;_%7B%28M%29%7Dy_%7BQualisys%2Cbase%7D)

Using the least_squares optimization method, six results for phi will be calculated, each with three x and y coordinates.

The determined transformation matrix is then checked with the known values. If the offset between transformed Qualisys coordinates and real MIR coordinates is very small, the transformation is saved in an array.

### - Usage
The MIR platform must move along a path while the Node is running. Excecuting the node will continously calculate transformation matrices and decide if they are accurate enough. After 200 good transformations, the node will stop and publish the median transformation to the terminal.