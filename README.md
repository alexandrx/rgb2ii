# RGB2II 

Converts an input RGB image to its Illumination Invariant (II) form.

![II](docs/rgb2ii.png?raw=:q:qtrue "RGB2II") 


## Credits 
This code is based on:
```
Maddern, W., Stewart, A., McManus, C., Upcroft, B., Churchill, W., & Newman, P. "Illumination invariant imaging: Applications in robust vision-based localisation, mapping and classification for autonomous vehicles." In Proceedings of the Visual Place Recognition in Changing Environments Workshop, IEEE International Conference on Robotics and Automation (ICRA), Hong Kong, China (Vol. 2, No. 3, p. 5), 2014.
```
[Paper](https://www.robots.ox.ac.uk/~mobile/Papers/2014ICRA_maddern.pdf)

## Install
-------
Please build this software using the following commands:
```
mkdir <ROS-WORKSPACE>/rgb2ii/src -p
cd <ROS-WORKSPACE>/rgb2ii/src
git clone <THIS-CODE-REPOSITORY>
cd ../
```
To build in ROS1 use:
```
catkin build  # <- you can also use: colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
or
```
catkin_make
```

## Usage
-----
Please run using the following commands:
```
cd <ROS-WORKSPACE>/rgb2ii
source install/setup.bash
python install/rgb2ii/lib/rgb2ii/rgb2ii.py image:=<INPUT IMAGE TOPIC> alpha:=<SOME ALPHA VALUE> 
```
Available arguments are as follows:

- **image:** topic name of the input image  (default: /image_raw)
- **alpha:** alpha value for the II transformation (default: 0.394)

You can use RVIZ to visualize both the input image and its corresponding II result.

## TODO
- [x] Add ROS support
- [ ] The method is quite heavy for its use of np.log of the image, other acceleration is necessary
- [ ] Create a database of alpha values for many cameras