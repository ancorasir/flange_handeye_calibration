import os
import ropsy

class Photoneo_Controller:
    def __init__(self, script_path):
        # Path of the script used to trigger camera
        self.script_path = script_path

    def get_image(self, im_name="tem_pl.ply"):
        """
        Shot an image with RGB-D camera
        :param im_name: name of the ply image
        :return:
        """
        status = os.system('sh ./%s %s' % (self.script_path, im_name))
        rospy.sleep(2)
        return status
