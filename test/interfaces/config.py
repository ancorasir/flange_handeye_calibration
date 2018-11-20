#!/usr/bin/env python

# camera ID for example "InstalledExamples-PhoXi-example" for file camera, "1711002" for real scanner
camera_id = "PhoXi3DScan-1711004"

node_name = "/phoxi_camera"

srv_timeout = 1

class topic:
    diagnostics         = "/diagnostics"
    confidence_map      = node_name + "/confidence_map"
    normal_map          = node_name + "/normal_map"
    param_description   = node_name + "/parameter_descriptions"
    param_update        = node_name + "/parameter_updates"
    point_cloud         = node_name + "/pointcloud"
    texture             = node_name + "/texture"

class service:
    connect_camera      = node_name + "/connect_camera"
    disconnect_camera   = node_name + "/disconnect_camera"
    get_device_list     = node_name + "/get_device_list"
    get_frame           = node_name + "/get_frame"
    get_hardware_indentification = node_name + "/get_hardware_indentification"
    get_loggers         = node_name + "/get_loggers"
    get_supported_capturing_modes = node_name + "/get_supported_capturing_modes"
    is_acquiring        = node_name + "/is_acquiring"
    is_connected        = node_name + "/is_connected"
    save_frame          = node_name + "/save_frame"
    set_logger_level    = node_name + "/set_logger_level"
    set_parameters      = node_name + "/set_parameters"
    start_acquisition   = node_name + "/start_acquisition"
    stop_acquisition    = node_name + "/stop_acquisition"
    trigger_image       = node_name + "/trigger_image"
    # V2
    V2_is_acquiring         = node_name + "/V2/is_acquiring"
    V2_is_connected         = node_name + "/V2/is_connected"
    V2_set_coordination_space = node_name + "/V2/set_coordination_space"
    V2_set_transformation   = node_name + "/V2/set_transformation"
    V2_start_acquisition    = node_name + "/V2/start_acquisition"
    V2_stop_acquisition     = node_name + "/V2/stop_acquisition"

class param:
    confidence          = node_name + "/confidence"
    coordination_space  = node_name + "/coordination_space"
    frame_id            = node_name + "/frame_id"
    resolution          = node_name + "/resolution"
    scan_multiplier     = node_name + "/scan_multiplier"
    scanner_id          = node_name + "/scanner_id"
    send_confidence_map = node_name + "/send_confidence_map"
    send_deapth_map     = node_name + "/send_deapth_map"
    send_normal_map     = node_name + "/send_normal_map"
    send_point_cloud    = node_name + "/send_point_cloud"
    send_texture        = node_name + "/send_texture"
    shutter_multiplier  = node_name + "/shutter_multiplier"
    timeout             = node_name + "/timeout"
    trigger_mode        = node_name + "/trigger_mode"
