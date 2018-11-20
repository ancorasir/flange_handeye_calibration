#!/usr/bin/env python

"""
this testing script calls all services of phoxi_camera node
"""
PKG = 'phoxi_camera'

from unittest import TestCase
from config import *
import rospy
import std_srvs.srv
import sensor_msgs.msg
from ros_utils import *
import phoxi_camera.srv as phoxi_camera_srv

published_topics_num = 0

def connect():
    rospy.wait_for_service(service.connect_camera)
    srv_connect = rospy.ServiceProxy(service.connect_camera, phoxi_camera_srv.ConnectCamera)
    srv_connect(camera_id)

def disconnect():
    rospy.wait_for_service(service.disconnect_camera)
    srv_disconnect = rospy.ServiceProxy(service.disconnect_camera, std_srvs.srv.Empty)
    srv_disconnect()

class Test_phoxi_camera_ros_interface(TestCase):
    def setUp(self):
        rospy.init_node('Test_ROS_interfaces')
        connect()

    def test_getDeviceList(self):
        srv = rospy.ServiceProxy(service.get_device_list, phoxi_camera_srv.GetDeviceList)
        response = srv()

        assert True == response.success
        assert "Ok" == response.message
        assert 0 != response.len
        print("Achievable cameras, you can check them, it must be equal to reality:")
        print(response.out)

    def test_connect_disconnectCamera(self):
        srv_connect = rospy.ServiceProxy(service.connect_camera, phoxi_camera_srv.ConnectCamera)
        srv_dicsonnect = rospy.ServiceProxy(service.disconnect_camera, std_srvs.srv.Empty)
        srv_connected = rospy.ServiceProxy(service.is_connected, phoxi_camera_srv.IsConnected)

        # connect
        response = srv_connect(camera_id)

        assert True == response.success
        assert "Ok" == response.message

        response = srv_connected()
        assert True == response.connected

        # disconnect
        srv_dicsonnect()

        response = srv_connected()
        assert False == response.connected

    def test_isConnected(self):
        # disconnected
        disconnect()
        srv_connected = rospy.ServiceProxy(service.is_connected, phoxi_camera_srv.IsConnected)
        res = srv_connected()

        assert False == res.connected

        # connected
        connect()
        res = srv_connected()

        assert True == res.connected

    def test_isConnectedV2(self):
        # disconnected
        disconnect()
        srv_connected = rospy.ServiceProxy(service.V2_is_connected, phoxi_camera_srv.GetBool)
        res = srv_connected()

        assert False == res.value
        assert True == res.success
        assert "Ok" == res.message

        # connected
        connect()
        res = srv_connected()

        assert True == res.value
        assert True == res.success
        assert "Ok" == res.message

    def test_isAcquiring(self):
        srv_acquiring = rospy.ServiceProxy(service.is_acquiring, phoxi_camera_srv.IsAcquiring)
        res = srv_acquiring()

        assert True == res.is_acquiring

        disconnect()
        res = srv_acquiring()

        assert False == res.is_acquiring

    def test_isAcquiringV2(self):
        srv_acquiring = rospy.ServiceProxy(service.V2_is_acquiring, phoxi_camera_srv.GetBool)
        res = srv_acquiring()

        assert True == res.value
        assert True == res.success
        assert "Ok" == res.message

        disconnect()
        res = srv_acquiring()

        assert False == res.value
        assert True == res.success
        assert "Ok" == res.message

    def test_startAcquisition(self):
        srv_startAcq = rospy.ServiceProxy(service.start_acquisition, std_srvs.srv.Empty)
        srv_startAcq()  # TODO no throw

    def test_stopAcquisition(self):
        srv_stopAcq = rospy.ServiceProxy(service.stop_acquisition, std_srvs.srv.Empty)
        srv_stopAcq()  # TODO no except

    def test_startAcquisitionV2(self):
        srv_startAcq = rospy.ServiceProxy(service.V2_start_acquisition, phoxi_camera_srv.Empty)
        res = srv_startAcq()  # TODO no throw

        assert True == res.success
        assert "Ok" == res.message

    def test_stopAcquisitionV2(self):
        srv_stopAcq = rospy.ServiceProxy(service.V2_stop_acquisition, phoxi_camera_srv.Empty)
        res = srv_stopAcq()  # TODO no except

        assert True == res.success
        assert "Ok" == res.message

    def test_triggerImage(self):
        srv_trig = rospy.ServiceProxy(service.trigger_image, phoxi_camera_srv.TriggerImage)
        res = srv_trig()

        assert True == res.success
        assert "Ok" == res.message

        # disconnected
        disconnect()
        res = srv_trig()

        assert False == res.success
        assert "Ok" != res.message

    def test_getFrame(self):
        global published_topics_num
        import time

        def callback(data):
            global published_topics_num
            published_topics_num = published_topics_num + 1

        # create subscribers
        rospy.Subscriber(topic.point_cloud, sensor_msgs.msg.PointCloud2, callback)
        rospy.Subscriber(topic.normal_map, sensor_msgs.msg.Image, callback)
        rospy.Subscriber(topic.confidence_map, sensor_msgs.msg.Image, callback)
        rospy.Subscriber(topic.texture, sensor_msgs.msg.Image, callback)

        srv_getFrame = rospy.ServiceProxy(service.get_frame, phoxi_camera_srv.GetFrame)
        res = srv_getFrame(-1)
        time.sleep(5)

        assert True == res.success
        assert "Ok" == res.message
        assert published_topics_num == 4, "Some topic was not published after get_frame service"

        # disconnected
        disconnect()
        res = srv_getFrame(-1)

        assert False == res.success
        assert "Ok" != res.message

    def test_saveFrame(self):
        import os
        path = os.getcwd()  # it should be ~/.ros
        filename = "/file.ply"
        os.system("rm " + path + filename)  # remove old file

        dir = os.listdir(path)

        srv_saveFrame = rospy.ServiceProxy(service.save_frame, phoxi_camera_srv.SaveFrame)
        res = srv_saveFrame(-1, path + filename)

        dir = set(os.listdir(path)) - set(dir)
        assert len(dir) != 0
        assert True == res.success
        assert "Ok" == res.message

        os.system("rm " + path + filename)  # remove created file

    def test_getHardwareIdentification(self):
        # connected
        srv_getHardware = rospy.ServiceProxy(service.get_hardware_indentification,
                                             phoxi_camera_srv.GetHardwareIdentification)
        res = srv_getHardware()

        assert camera_id == res.hardware_identification
        assert True == res.success
        assert "Ok" == res.message

        # disconnect
        disconnect()
        res = srv_getHardware()

        assert "" == res.hardware_identification
        assert False == res.success
        assert "Ok" != res.message

    def test_getSupportedCapturingModes(self):
        # connected
        srv_getModes = rospy.ServiceProxy(service.get_supported_capturing_modes,
                                          phoxi_camera_srv.GetSupportedCapturingModes)
        res = srv_getModes()

        assert len(res.supported_capturing_modes) != 0
        assert True == res.success
        assert "Ok" == res.message

        # disconnect
        disconnect()
        res = srv_getModes()

        assert len(res.supported_capturing_modes) == 0
        assert False == res.success
        assert "Ok" != res.message

    def test_setCoordianteSpace(self):
        srv_setSpace = rospy.ServiceProxy(service.V2_set_coordination_space, phoxi_camera_srv.SetCoordinatesSpace)

        # 0 - 5 c++ enum, pho::api::PhoXiCoordinateSpace::...
        res = srv_setSpace(0)
        assert True == res.success
        assert "Ok" == res.message

        res = srv_setSpace(1)
        assert True == res.success
        assert "Ok" == res.message

        res = srv_setSpace(2)
        assert True == res.success
        assert "Ok" == res.message

        res = srv_setSpace(3)
        assert True == res.success
        assert "Ok" == res.message

        res = srv_setSpace(4)
        assert True == res.success
        assert "Ok" == res.message

        res = srv_setSpace(5)
        assert True == res.success
        assert "Ok" == res.message

    def test_setTransformation(self):
        srv_transform = rospy.ServiceProxy(service.V2_set_transformation, phoxi_camera_srv.SetTransformationMatrix)

        res = srv_transform([], 5, True, False)
        assert False == res.success
        assert "Bad matrix dimensions!" == res.message

        res = srv_transform([1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16], 5, True, False)
        assert True == res.success
        assert "Ok" == res.message

    def test_topics_running(self):
        """
        test if there are all the necessary topics that have been published
        """

        assert topic_is_running(topic.confidence_map) == True, \
            "Topic %s, not exist" % (topic.confidence_map)

        assert topic_is_running(topic.diagnostics) == True, \
            "Topic %s, not exist" % (topic.diagnostics)

        assert topic_is_running(topic.normal_map) == True, \
            "Topic %s, not exist" % (topic.normal_map)

        assert topic_is_running(topic.param_description) == True, \
            "Topic %s, not exist" % (topic.param_description)

        assert topic_is_running(topic.param_update) == True, \
            "Topic %s, not exist" % (topic.param_update)

        assert topic_is_running(topic.point_cloud) == True, \
            "Topic %s, not exist" % (topic.point_cloud)

        assert topic_is_running(topic.texture) == True, \
            "Topic %s, not exist" % (topic.texture)

    def test_parameter_server_variables_exist(self):
        """
        test if variables exist in parameter server
        """

        assert rospy.has_param(param.confidence) == True, \
            "Parameter %s is not exist" % param.confidence

        assert rospy.has_param(param.coordination_space) == True, \
            "Parameter %s is not exist" % param.coordination_space

        assert rospy.has_param(param.frame_id) == True, \
            "Parameter %s is not exist" % param.frame_id

        assert rospy.has_param(param.resolution) == True, \
            "Parameter %s is not exist" % param.resolution

        assert rospy.has_param(param.scan_multiplier) == True, \
            "Parameter %s is not exist" % param.scan_multiplier

        assert rospy.has_param(param.scanner_id) == True, \
            "Parameter %s is not exist" % param.scanner_id

        assert rospy.has_param(param.send_confidence_map) == True, \
            "Parameter %s is not exist" % param.send_confidence_map

        assert rospy.has_param(param.send_deapth_map) == True, \
            "Parameter %s is not exist" % param.send_deapth_map

        assert rospy.has_param(param.send_normal_map) == True, \
            "Parameter %s is not exist" % param.send_normal_map

        assert rospy.has_param(param.send_point_cloud) == True, \
            "Parameter %s is not exist" % param.send_point_cloud

        assert rospy.has_param(param.send_texture) == True, \
            "Parameter %s is not exist" % param.send_texture

        assert rospy.has_param(param.shutter_multiplier) == True, \
            "Parameter %s is not exist" % param.shutter_multiplier

        assert rospy.has_param(param.timeout) == True, \
            "Parameter %s is not exist" % param.timeout

        assert rospy.has_param(param.trigger_mode) == True, \
            "Parameter %s is not exist" % param.trigger_mode

if __name__ == '__main__':
    import rostest

    rostest.rosrun(PKG, 'Test_ROS_interface_class', Test_phoxi_camera_ros_interface)