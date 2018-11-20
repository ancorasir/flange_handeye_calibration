#!/usr/bin/env python

"""
this script tests base interfaces of phoxi_camera node
"""
PKG = 'phoxi_camera'

from unittest import TestCase
from config import *
import rospy
import phoxi_camera.srv as phoxi_camera_srv
from ros_utils import *

def connect():
    rospy.wait_for_service(service.connect_camera)
    srv_connect = rospy.ServiceProxy(service.connect_camera, phoxi_camera_srv.ConnectCamera)
    srv_connect(camera_id)

class Test_phoxi_camera_interfaces(TestCase):
    def setUp(self):
        rospy.init_node('Test_ROS_interfaces')
        connect()

    def test1_phoxi_camera_node_exist(self):
        """
        test if phoxi_camera node running
        """

        assert node_is_running(node_name) == True, "Node %s is not running" % node_name

    def test2_connect_camera(self):
        """
        test connect to a scanner
        """

        try:
            rospy.wait_for_service(service.connect_camera, srv_timeout)
            connect_camera = rospy.ServiceProxy(service.connect_camera, phoxi_camera_srv.ConnectCamera)

            response = connect_camera(camera_id)
            assert response.success == True, "Connection to the camera was unsuccessful, success != True"
        except rospy.ROSException as e:
            self.fail("Service %s not exist, %s " % (service.connect_camera, e))
        except rospy.ServiceException as e:
            self.fail("Service %s call failed, %s" % (service.connect_camera, e))

    def test3_topics_running(self):
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

    def test4_services_running(self):
        """
        test if there are all the necessary services that have been created
        """

        assert service_is_running(service.connect_camera) == True, \
            "Service %s is not exist" % service.connect_camera

        assert service_is_running(service.disconnect_camera) == True, \
            "Service %s is not exist" % service.disconnect_camera

        assert service_is_running(service.get_device_list) == True, \
            "Service %s is not exist" % service.get_device_list

        assert service_is_running(service.get_frame) == True, \
            "Service %s is not exist" % service.get_frame

        assert service_is_running(service.get_hardware_indentification) == True, \
            "Service %s is not exist" % service.get_hardware_indentification

        assert service_is_running(service.get_loggers) == True, \
            "Service %s is not exist" % service.get_loggers

        assert service_is_running(service.get_supported_capturing_modes) == True, \
            "Service %s is not exist" % service.get_supported_capturing_modes

        assert service_is_running(service.is_acquiring) == True, \
            "Service %s is not exist" % service.is_acquiring

        assert service_is_running(service.is_connected) == True, \
            "Service %s is not exist" % service.is_connected

        assert service_is_running(service.save_frame) == True, \
            "Service %s is not exist" % service.save_frame

        assert service_is_running(service.set_logger_level) == True, \
            "Service %s is not exist" % service.set_logger_level

        assert service_is_running(service.set_parameters) == True, \
            "Service %s is not exist" % service.set_parameters

        assert service_is_running(service.start_acquisition) == True, \
            "Service %s is not exist" % service.start_acquisition

        assert service_is_running(service.stop_acquisition) == True, \
            "Service %s is not exist" % service.stop_acquisition

        assert service_is_running(service.trigger_image) == True, \
            "Service %s is not exist" % service.trigger_image

        assert service_is_running(service.V2_is_acquiring) == True, \
            "Service %s is not exist" % service.V2_is_acquiring

        assert service_is_running(service.V2_is_connected) == True, \
            "Service %s is not exist" % service.V2_is_connected

        assert service_is_running(service.V2_set_coordination_space) == True, \
            "Service %s is not exist" % service.V2_set_coordination_space

        assert service_is_running(service.V2_set_transformation) == True, \
            "Service %s is not exist" % service.V2_set_transformation

        assert service_is_running(service.V2_start_acquisition) == True, \
            "Service %s is not exist" % service.V2_start_acquisition

        assert service_is_running(service.V2_stop_acquisition) == True, \
            "Service %s is not exist" % service.V2_stop_acquisition

    def test5_parameter_server_variables_exist(self):
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

    rostest.rosrun(PKG, 'Test_interfaces_exist', Test_phoxi_camera_interfaces)