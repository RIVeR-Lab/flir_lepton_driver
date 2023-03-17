# FLIR Lepton Driver

A fully ROS compatible for the FLIR Lepton Long Wave Infrared Thermal Camera.

Tested with the [PureThermal2](https://groupgets.com/manufacturers/getlab/products/purethermal-2-flir-lepton-smart-i-o-module) module from GroupGets.

### Building

```
cd <<catkin_ws/src>>
git clone https://github.com/RIVeR-Lab/flir_lepton_driver.git
catkin build
source devel/setup.bash
```

### Running the Driver
```
roslaunch flir_lepton_driver thermal_img_pub.launch
```