import os


class Photoneo_Controller:
    def __init__(self, script_path):
        # Path of the script used to trigger camera
        self.script_path = script_path

    def get_image(self, point_cloud_dir):
        """
        Shot an image with RGB-D camera
        :param point_cloud_dir: name of the stored ply image
        :return:
        """
        ret = os.system(self.script_path + r' ' + point_cloud_dir)
        # status = os.system('sh ./%s %s' % (self.script_path, im_name))
        return ret
