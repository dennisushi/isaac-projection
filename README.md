# Demo for Projective Geometry in IsaacSim

Sample util functions for projection of a pinhole camera pixel UV to world frame XYZ, and vice versa.

Specific to IsaacSim since (i) the depth input is inversed, i.e. `(1/depth)`, (ii) it uses the ViewProjection matrix, (iii) the camera axes are permuted and (iv) the pinhole camera is using a frensel model with near-far limits.

To run demo, simply `python3 demo.py`

The following images show the 3D points projected into 2D (left), then projected to 3D and back to 2D (rigt).

![image](https://user-images.githubusercontent.com/30011340/178758806-c9553f0f-a0b4-492d-9a8b-765463877e08.png)
![image](https://user-images.githubusercontent.com/30011340/178758878-26139d5f-ec77-4ea5-8df0-961890e01330.png)


For the leftmost point shown, the error in estimation is up to 0.2cm (or 0.002m) in each axis.
```
XYZ_true = 
[[100  0   0]]
XYZ_est = 
[[100.14210758  -0.10772588   0.09006064]]
```
