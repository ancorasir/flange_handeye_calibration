#!/usr/bin/env python

def node_is_running(node_name):
    import rosnode

    try:
        nodes = rosnode.get_node_names()
        for n in nodes:
            if node_name in n:
                return True
        return False
    except:
        return False

def service_is_running(service_name):
    import rospy
    from config import srv_timeout

    try:
        rospy.wait_for_service(service_name, srv_timeout)
        return True
    except:
        return False

def topic_is_running(topic_name):
    import rospy

    try:
        topics = rospy.get_published_topics()
        for t in topics:
            if topic_name in t:
                return True
        return False
    except:
        return False


def assert_call_service(name, service_class):
    import rospy
    from pytest import fail

    try:
        rospy.wait_for_service(name, 1)
        service = rospy.ServiceProxy(name, service_class)
        response = service()

    except rospy.ROSException as e:
        fail("Service %s not exist, %s " % (name, e))

    except rospy.ServiceException as e:
        fail("Service %s call failed, %s" % (name, e))