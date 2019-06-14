import os


class Photoneo_Controller:
    def __init__(self, cfg):
        # Path of the script used to trigger camera
        self.script_path = cfg.CAM_CONTROLLER.SCRIPT_PATH

    def get_image(self, point_cloud_dir):
        """
        Shot an image with RGB-D camera
        :param point_cloud_dir: name of the stored ply image
        :return:
        """
        ret = 256
        attemps = 0
        while ret != 0 and attemps < 3:
            ret = os.system(self.script_path + r' ' + point_cloud_dir)
        return ret
