# Flanged based hand-eye calibration calibration toolbox

Flanged based hand-eye calibration is a method for hand-eye calibration between a robot arm and a 3D scanner based on the geometric features of the tool flange of the robot arm. The method depends on the high precision point cloud provided by the 3D scanner.

This repository provides a toolbox written in python executing the hand-eye calibration process. The toolbox has been test on robot arm including Franka, UR5, UR5e, UR10e, Aubo-i5 and 3D scanners including Phoxi-S, Phoxi-M, .

## Overview of method

![module_design](./img/img_doc.png)

To support more robot & camera & calibration algorithms, just implement interfaces. Please refer to the [readme](./calib_toolbox/README.md) for a detailed description.

## Installation Dependencies

- `numpy`
- `yacs`
- `transforms3d`
- `open3d`
- `[Phoxi Control v1.2.7](https://www.photoneo.com/3d-scanning-software/)`
- `Franka_ROS package`


## Example code

### Data collection & online callibtraion

**Related file:** `/calib_toolbox/scripts/calib.py`

**Related file:**`/calib_toolbox/config_files/online_calib.yaml`

### Offline calibration

**Related file:**`/calib_toolbox/scripts/test.py`

**Related file:**`/calib_toolbox/config_files/offline_calib.yaml`

### Notification

- Data is stored in original unit. Be careful of the unit when apply calibration.

- Use config file instead of hardcode

- Check ‘calib_toolbox.config.defaults.py’ to see all config parameters or add your own config.
