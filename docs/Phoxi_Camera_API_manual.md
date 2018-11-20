# Phoxi_Camera Service API 

## Service 

### connect_camera

**Description:** Start *Phoxi_Control* and connect to the camera with required id. Suggest to sleep for 2 seconds to get camera connected. 

**Input:** 

string name: id of required camera

**Output:**

string message : indication of status; 'OK', 'Disconnected', 'Scanner not found'

bool success:



### get_device_list
**Description:**

**Input:** NaN

**Output:** 

int64 len: number of allowed cameras

string[] out: list of id of cameras

string message: indication of status

bool success:

### get_frame
**Description:** Get a frame and publish it

**Input:** int64 in : id of Input frame; negative number means get a new frame 

**Output:**

String message: indication of status

bool success

### save_frame
**Description:**

**Input:**

int64 in: id of scan returned by **trigger_image**, set it to negative if want to take a new image

string path:

**Output:**

string message: indication of status

bool success



### trigger_image
**Description:** Get an image from camera

**Input:** NaN

**Output:**

int32 id: id of the scan

string message: indicating status

bool success: boolean value indicate if success

### set_coordinate_space

refer to **set_transformation_matrix**

### set_transformation_matrix

**Description:** Apply transformation matrix to the camera. Dangerous to set value directly to the camera, better not use. 

**Input:**

geometry_msg/Transform transform: applied transform 

guint8 coordinate_space: 1 = CameraSpace, 2 = MountingSpace, 3 = MarkerSpace, 4 = RobotSpace, 5 = CustomSpace

bool set_value: if true then set coordinate_space to PhoXi Control

bool save_settings: if true then keep the settings 

**Output:**

string message: indication of status 

bool success



## Topics


### /phoxi_camera/confidence_map

**Description:** 

**Message Type:**

### /phoxi_camera/depth_map

**Description:** Depth image

**Message Type:** sensor_msgs/Image(Header, height, width, encoding, step, data[])

### /phoxi_camera/normal_map

**Description:** 

**Message Type:**

### /phoxi_camera/parameter_descriptions

**Description:** 

**Message Type:**

### /phoxi_camera/parameter_updates

**Description:** 

**Message Type:**

### /phoxi_camera/pointcloud

**Description:** Point cloud image

**Message Type:** sensor_msgs/PointCloud2 (Header, height, width, fields[], point_step, row_step, is_dense, data[] )

### /phoxi_camera/rgb_texture

**Description:** RGB image

**Message Type:** sensor_msgs/Image(Header, height, width, encoding, step, data[])

### /phoxi_camera/texture

**Description:** Gray scale image

**Message Type:** sensor_msgs/Image (Header, height, width, encoding, step, data[])