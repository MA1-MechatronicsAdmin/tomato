# tomato2025

## Projects
- dynamixelcontrol
- tomato_dynamixel
- tomato_pubsub
- tomato_joycon
- tomato_kobuki
- tomato_dynamixel
- yolo_detection

## how to build the projects

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone 
$ cd tomato2023
$ catkin build
```

## Updates
- removed y axis movement
- added restriction to x axis movement
- Realsense and yolov8 for tomato positioning
- Monocular camera also supported
- Added "start" button functionality for toggle between autonomous mode and controlled mode
- Added "back" button functionality for resetting position of arm

## Basic Arm geometry
### x, y coordinates
![equation](https://latex.codecogs.com/svg.image?&space;x=l(sin(\theta_1)&plus;sin(\theta_2)))


![equation](https://latex.codecogs.com/svg.image?&space;y=l(cos(\theta_1)-cos(\theta_2)))

### inverse kinematics solution 
![equation](https://latex.codecogs.com/svg.image?\theta_1=sin^{-1}(\frac{\sqrt{x^2&plus;y^2}}{2l})&plus;tan^{-1}(\frac{y}{x}))


![equation](https://latex.codecogs.com/svg.image?\theta_1=sin^{-1}(\frac{\sqrt{x^2&plus;y^2}}{2l})-tan^{-1}(\frac{y}{x}))

## Usage (code)
```cpp
bool paused; // for pausing
bool backed; // for reset position
bool autonomous; // auto mode

void go_to(double _x, double _y, _double _z, ...){
...
}

// arm goes to (_x, _y, _z)

void make_move(...){
...
}

// make updates to robot movement (target_val is radians)
```

## How to launch
### Base file
```bash
$ roslaunch dynamixelcontrol param.launch
```
### Camera-based tomato positioning 
```bash
$ roslaunch dynamixelcontrol follow.launch
```
### Realsense camera-based tomato positioning 
```bash
$ roslaunch dynamixelcontrol real.launch
```
